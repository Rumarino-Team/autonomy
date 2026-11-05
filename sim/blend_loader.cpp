// Blender evaluates .blend data; only the immutable binary scene crosses IPC.
#include "image.hpp"
#include <filesystem>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <dlfcn.h>
extern char **environ;
namespace sim {
namespace {
constexpr char magic_bytes[] = {'A', 'S', 'I', 'M', 'S', 'C', '0', '2'};
struct FileDescriptor {
  int fd = -1;
  ~FileDescriptor() { if (fd >= 0) ::close(fd); }
};
struct CloseFile {
  void operator()(FILE *file) const { std::fclose(file); }
};
const char *option(const char *key, const char *fallback) {
  auto value = std::getenv(key);
  return value && *value ? value : fallback;
}
void read_exact(int fd, void *data, size_t count,
                std::chrono::steady_clock::time_point deadline) {
  auto out = static_cast<char *>(data);
  while (count) {
    auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline-std::chrono::steady_clock::now()).count();
    require(remaining > 0, "Blender scene transfer timed out");
    pollfd p{fd, POLLIN, 0};
    int ready = ::poll(&p, 1, int(remaining));
    if (ready < 0 && errno == EINTR) continue;
    require(ready > 0, "Blender scene transfer timed out or failed");
    ssize_t n = ::read(fd, out, count);
    if (n < 0 && errno == EINTR) continue;
    require(n > 0, "truncated binary scene transfer");
    out += n; count -= n;
  }
}
}
Scene Scene::load_fd(int fd) {
  require(fd >= 0, "invalid scene descriptor");
  auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(120);
  unsigned char header[16];
  read_exact(fd, header, sizeof(header), deadline);
  require(std::memcmp(header, magic_bytes, 8) == 0, "unsupported binary scene version");
  uint64_t count = 0;
  for (int i=0;i<8;++i) count |= uint64_t(header[8+i]) << (8*i);
  require(count && count <= max_scene_bytes, "invalid binary scene size");
  struct stat status{};
  if(::fstat(fd,&status)==0 && S_ISREG(status.st_mode))
    require(uint64_t(status.st_size)==16+count,"binary scene file size does not match its header");
  std::vector<uint8_t> file(16 + count);
  std::memcpy(file.data(), header, 16);
  read_exact(fd, file.data() + 16, count, deadline);
  return Scene::read(file.data(), file.size());
}
Scene Scene::load_blend(const std::string &path) {
  require(std::filesystem::is_regular_file(path),"Blender scene file is missing");
  const char *frame = option("AUTONOMY_SIM_SOURCE_FRAME", "1");
  uint32_t value=0; auto end=frame+std::strlen(frame);
  auto [last,error]=std::from_chars(frame,end,value);
  require(error==std::errc() && last==end && value<=2147483647, "invalid Blender source frame");
  // Installed bundles carry the same evaluated loader beside their runtime.
  std::filesystem::path bundled;
  Dl_info library{};
  if(dladdr(reinterpret_cast<void *>(&Scene::load_blend),&library) && library.dli_fname)
    bundled=std::filesystem::path(library.dli_fname).parent_path().parent_path()/"share/autonomy/blender/export_scene.py";
  auto default_script=std::filesystem::is_regular_file(bundled) ? bundled.string() : std::string(SIM_BLENDER_EXPORT_SCRIPT);
  const char *script = option("AUTONOMY_SIM_BLENDER_SCRIPT", default_script.c_str());
  require(std::filesystem::is_regular_file(script), "Blender scene loader script is missing; set AUTONOMY_SIM_BLENDER_SCRIPT");
  int pair[2]; require(::socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, pair)==0, "cannot create Blender scene socket");
  FileDescriptor reader{pair[0]}, writer{pair[1]};
  auto log=std::unique_ptr<FILE, CloseFile>(std::tmpfile());
  require(bool(log), "cannot create Blender diagnostic log");
  ::fcntl(fileno(log.get()), F_SETFD, FD_CLOEXEC);
  std::string file=std::filesystem::absolute(path).string();
  std::vector<std::string> arguments={option("AUTONOMY_SIM_BLENDER_EXECUTABLE","blender"),
      "--background","--factory-startup","--disable-autoexec",file,
      "--python-exit-code","1","--python",script,"--","--scene",
      option("AUTONOMY_SIM_SOURCE_SCENE","AutonomySource"),"--frame",frame,
      "--profiles",option("AUTONOMY_SIM_PROFILES",SIM_BLENDER_PROFILES),"--pipe-fd","3"};
  std::vector<char *> argv;
  for(auto &a:arguments) argv.push_back(a.data());
  argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  require(posix_spawn_file_actions_init(&actions)==0,"cannot prepare Blender process");
  posix_spawn_file_actions_adddup2(&actions,writer.fd,3);
  posix_spawn_file_actions_adddup2(&actions,fileno(log.get()),STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions,fileno(log.get()),STDERR_FILENO);
  pid_t pid=-1;
  int spawned=posix_spawnp(&pid,argv[0],&actions,nullptr,argv.data(),environ);
  posix_spawn_file_actions_destroy(&actions);
  require(spawned==0,"cannot start Blender; set AUTONOMY_SIM_BLENDER_EXECUTABLE");
  ::close(writer.fd); writer.fd=-1;
  auto diagnostics=[&] {
    std::fflush(log.get());
    std::fseek(log.get(),0,SEEK_END); long length=std::ftell(log.get());
    std::fseek(log.get(),std::max(0L,length-8192),SEEK_SET);
    char buffer[8193]{}; auto n=std::fread(buffer,1,8192,log.get());
    return std::string(buffer,n);
  };
  try {
    auto scene=load_fd(reader.fd);
    int status=0;
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(15);
    while(true) {
      auto result=::waitpid(pid,&status,WNOHANG);
      if(result==pid) { pid=-1; break; }
      if(result<0 && errno!=EINTR) throw std::runtime_error("cannot wait for Blender process");
      require(std::chrono::steady_clock::now()<deadline,"Blender did not exit after scene transfer");
      ::poll(nullptr,0,10);
    }
    require(WIFEXITED(status) && WEXITSTATUS(status)==0,"Blender scene evaluation failed");
    scene.path=std::filesystem::absolute(path).string();
    return scene;
  } catch(const std::exception &error) {
    if(pid>0) { ::kill(pid,SIGKILL); while(::waitpid(pid,nullptr,0)<0 && errno==EINTR) {} }
    throw std::runtime_error(std::string(error.what())+"\n"+diagnostics());
  }
}
} // namespace sim

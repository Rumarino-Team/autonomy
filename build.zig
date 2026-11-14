const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});

    const optimize = b.standardOptimizeOption(.{});
    const cxx = b.option([]const u8, "cxx", "System C++ compiler") orelse "g++";
    const zed_prefix = b.option([]const u8, "zed-prefix", "ZED SDK install prefix") orelse "/usr/local/zed";
    const cuda_prefix = b.option([]const u8, "cuda-prefix", "CUDA install prefix") orelse "/usr/local/cuda";
    const stdcxx_path = b.option(
        []const u8,
        "stdcxx-path",
        "System libstdc++.so used by ZED C++ objects",
    ) orelse "/usr/lib/x86_64-linux-gnu/libstdc++.so.6";

    const exe = b.addExecutable(.{
        .name = "src_3",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{},
        }),
    });

    exe.root_module.link_libc = true;
    exe.root_module.addIncludePath(b.path("include"));
    exe.root_module.addCSourceFile(.{
        .file = b.path("auvs/trap_sim/auv.c"),
    });
    b.installArtifact(exe);
    const mission_io = b.createModule(.{
        .root_source_file = b.path("src/MissionIO.zig"),
        .target = target,
        .optimize = optimize,
    });
    mission_io.addIncludePath(b.path("include"));

    const raylib_dep = b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    });
    const viewer_mod = b.createModule(.{
        .root_source_file = b.path("tools/viewer/main.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .imports = &.{
            .{ .name = "raylib", .module = raylib_dep.module("raylib") },
            .{ .name = "MissionIO", .module = mission_io },
        },
    });
    viewer_mod.linkLibrary(raylib_dep.artifact("raylib"));
    const viewer = b.addExecutable(.{
        .name = "viewer",
        .root_module = viewer_mod,
    });

    const viewer_zed = b.option(bool, "viewer-zed", "Enable ZED stream/SVO video in the Zig viewer") orelse false;
    const viewer_gstreamer = b.option(bool, "viewer-gstreamer", "Enable standard RTP video in the viewer using GStreamer") orelse false;
    if (viewer_zed and viewer_gstreamer) @panic("-Dviewer-zed and -Dviewer-gstreamer select different video backends");
    if (viewer_zed) {
        const compile_video = b.addSystemCommand(&.{
            cxx,
            "-c",
            "-fPIC",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
        });
        compile_video.addArgs(&.{
            "-isystem",
            b.fmt("{s}/include", .{zed_prefix}),
            "-isystem",
            b.fmt("{s}/include", .{cuda_prefix}),
            "-o",
        });
        const video_object = compile_video.addOutputFileArg("viewer_video.o");
        compile_video.addFileArg(b.path("tools/viewer/video.cpp"));
        viewer.root_module.addObjectFile(video_object);
        viewer.root_module.addLibraryPath(.{ .cwd_relative = b.fmt("{s}/lib", .{zed_prefix}) });
        viewer.root_module.addLibraryPath(.{ .cwd_relative = b.fmt("{s}/lib64", .{cuda_prefix}) });
        viewer.root_module.linkSystemLibrary("sl_zed", .{});
        viewer.root_module.linkSystemLibrary("cuda", .{});
        viewer.root_module.linkSystemLibrary("cudart", .{});
        viewer.root_module.linkSystemLibrary("gcc_s", .{ .use_pkg_config = .no });
        viewer.root_module.addObjectFile(.{ .cwd_relative = stdcxx_path });
    } else if (viewer_gstreamer) {
        viewer.root_module.link_libcpp = true;
        viewer.root_module.addCSourceFile(.{
            .file = b.path("tools/viewer/video_gstreamer.cpp"),
            .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" },
        });
        viewer.root_module.linkSystemLibrary("gstreamer-app-1.0", .{ .use_pkg_config = .force });
        viewer.root_module.linkSystemLibrary("gstreamer-video-1.0", .{ .use_pkg_config = .force });
    } else {
        viewer.root_module.addCSourceFile(.{
            .file = b.path("tools/viewer/video_stub.cpp"),
            .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" },
        });
        viewer.root_module.linkSystemLibrary("stdc++", .{});
    }

    const install_viewer = b.addInstallArtifact(viewer, .{});
    const viewer_step = b.step("viewer", "Build the Zig telemetry viewer");
    viewer_step.dependOn(&install_viewer.step);
    const run_step = b.step("run", "Run the app");
    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);

    run_cmd.step.dependOn(b.getInstallStep());

    run_cmd.addPassthruArgs();

    const exe_tests = b.addTest(.{
        .root_module = exe.root_module,
    });

    // A run step that will run the second test executable.
    const run_exe_tests = b.addRunArtifact(exe_tests);

    // A top level step for running all tests. dependOn can be called multiple
    // times and since the two run steps do not depend on one another, this will
    // make the two of them run in parallel.
    const test_step = b.step("test", "Run tests");
    // test_step.dependOn(&run_mod_tests.step);
    test_step.dependOn(&run_exe_tests.step);

    const viewer_ui_test_mod = b.createModule(.{
        .root_source_file = b.path("tools/viewer/ui.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .imports = &.{
            .{ .name = "raylib", .module = raylib_dep.module("raylib") },
            .{ .name = "MissionIO", .module = mission_io },
        },
    });
    viewer_ui_test_mod.linkLibrary(raylib_dep.artifact("raylib"));
    const viewer_ui_tests = b.addTest(.{ .root_module = viewer_ui_test_mod });
    const run_viewer_ui_tests = b.addRunArtifact(viewer_ui_tests);
    test_step.dependOn(&run_viewer_ui_tests.step);

    const map_tests = b.addExecutable(.{
        .name = "object_map_test",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libc = true,
            .link_libcpp = true,
        }),
    });
    map_tests.root_module.addCSourceFile(.{ .file = b.path("tests/object_map_test.cpp"), .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" } });
    map_tests.root_module.addCSourceFile(.{ .file = b.path("auvs/proteus_hwd/object_map.cpp"), .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" } });
    const run_map_tests = b.addRunArtifact(map_tests);
    test_step.dependOn(&run_map_tests.step);

    if (viewer_gstreamer) {
        const rtp_test_mod = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libc = true,
            .link_libcpp = true,
        });
        for ([_][]const u8{ "tests/rtp_stream_test.cpp", "tools/viewer/video_gstreamer.cpp", "auvs/proteus_hwd/rtp_relay.cpp" }) |path| {
            rtp_test_mod.addCSourceFile(.{ .file = b.path(path), .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" } });
        }
        rtp_test_mod.linkSystemLibrary("gstreamer-app-1.0", .{ .use_pkg_config = .force });
        rtp_test_mod.linkSystemLibrary("gstreamer-video-1.0", .{ .use_pkg_config = .force });
        const rtp_tests = b.addExecutable(.{ .name = "rtp_stream_test", .root_module = rtp_test_mod });
        const run_rtp_tests = b.addRunArtifact(rtp_tests);
        test_step.dependOn(&run_rtp_tests.step);
    }

    const arduino_cli = b.option([]const u8, "arduino-cli", "Arduino CLI executable") orelse "arduino-cli";
    const arduino_fqbn = b.option([]const u8, "arduino-fqbn", "Arduino board FQBN") orelse "arduino:avr:uno";
    const arduino_compile = b.addSystemCommand(&.{ arduino_cli, "compile", "--fqbn", arduino_fqbn });
    arduino_compile.addDirectoryArg(b.path("arduino/sketches/Proteus"));
    // Incremental watch for Arduino sketch edits (API varies by Zig version).
    if (@hasDecl(std.Build.Step, "addDirectoryWatchInput")) {
        _ = try arduino_compile.step.addDirectoryWatchInput(b.path("arduino/sketches/Proteus"));
    }

    const arduino_step = b.step("arduino", "Compile the Proteus Arduino sketch");
    arduino_step.dependOn(&arduino_compile.step);

    const arduino_sender = b.addExecutable(.{
        .name = "arduino_sender",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
        }),
    });
    arduino_sender.root_module.link_libc = true;
    arduino_sender.root_module.link_libcpp = true;
    arduino_sender.root_module.addCSourceFile(.{
        .file = b.path("auvs/proteus_hwd/arduino_sender.cpp"),
        .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" },
    });
    arduino_sender.root_module.addCSourceFile(.{
        .file = b.path("auvs/proteus_hwd/arduino.cpp"),
        .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" },
    });
    const install_arduino_sender = b.addInstallArtifact(arduino_sender, .{});
    b.getInstallStep().dependOn(&install_arduino_sender.step);

    const arduino_sender_step = b.step("arduino-sender", "Build the Arduino sender executable");
    arduino_sender_step.dependOn(&install_arduino_sender.step);

    const lib_auv_stationary_sim = b.addLibrary(.{
        .name = "auv_stationary_sim",
        .root_module = b.createModule(.{
            .target = b.graph.host,
            .optimize = optimize,
        }),
        .linkage = .dynamic,
    });
    lib_auv_stationary_sim.root_module.link_libc = true;
    lib_auv_stationary_sim.root_module.addCSourceFile(.{
        .file = b.path("auvs/stationary_sim/auv.c"),
        .flags = &.{"-fPIC"},
    });
    b.installArtifact(lib_auv_stationary_sim);

    const zed = b.option(bool, "zed", "Build the ZED camera plugin (requires ZED SDK and CUDA)") orelse false;
    const zed_rtp = b.option(bool, "zed-rtp", "Enable standard RTP relay in the ZED plugin (SDK >= 5.4 and GStreamer)") orelse false;
    if (zed_rtp and !zed) @panic("-Dzed-rtp requires -Dzed");
    if (zed) {
        const lib_auv_proteus_hwd = b.addSystemCommand(&.{cxx});
        lib_auv_proteus_hwd.addArgs(&.{ "-shared", "-fPIC", "-std=c++17", "-Wall", "-Wextra", "-Werror" });
        lib_auv_proteus_hwd.addArg(if (optimize == .debug) "-O0" else "-O2");
        if (optimize == .debug)
            lib_auv_proteus_hwd.addArg("-g");

        lib_auv_proteus_hwd.addArg("-o");
        const lib_auv_proteus_hwd_so = lib_auv_proteus_hwd.addOutputFileArg("libauv_proteus_hwd.so");
        if (zed_rtp) {
            lib_auv_proteus_hwd.addArg("-DAUV_STANDARD_RTP");
            lib_auv_proteus_hwd.addFileArg(b.path("auvs/proteus_hwd/rtp_relay.cpp"));
            lib_auv_proteus_hwd.addFileInput(b.path("auvs/proteus_hwd/rtp_relay.hpp"));
        }
        lib_auv_proteus_hwd.addFileArg(b.path("auvs/proteus_hwd/auv.cpp"));
        lib_auv_proteus_hwd.addFileArg(b.path("auvs/proteus_hwd/object_map.cpp"));
        lib_auv_proteus_hwd.addFileArg(b.path("auvs/proteus_hwd/arduino.cpp"));
        lib_auv_proteus_hwd.addFileInput(b.path("auvs/proteus_hwd/object_map.hpp"));
        lib_auv_proteus_hwd.addFileInput(b.path("auvs/proteus_hwd/coordinates.hpp"));
        lib_auv_proteus_hwd.addFileInput(b.path("auvs/proteus_hwd/arduino.hpp"));
        lib_auv_proteus_hwd.addFileInput(b.path("include/auv.h"));
        lib_auv_proteus_hwd.addFileInput(b.path("include/object_classes.h"));
        lib_auv_proteus_hwd.addFileInput(b.path("include/math.h"));
        lib_auv_proteus_hwd.addArgs(&.{
            "-isystem",
            b.fmt("{s}/include", .{zed_prefix}),
            "-isystem",
            b.fmt("{s}/include", .{cuda_prefix}),
            b.fmt("-L{s}/lib", .{zed_prefix}),
            b.fmt("-L{s}/lib64", .{cuda_prefix}),
            b.fmt("-Wl,-rpath,{s}/lib", .{zed_prefix}),
            b.fmt("-Wl,-rpath,{s}/lib64", .{cuda_prefix}),
            "-Wl,--no-undefined",
            "-lsl_zed",
            "-lcudart",
            "-lcuda",
            "-pthread",
        });

        if (zed_rtp) {
            const flags = b.run(&.{ "pkg-config", "--cflags", "--libs", "gstreamer-app-1.0" });
            var parts = std.mem.tokenizeAny(u8, flags, " \t\r\n");
            while (parts.next()) |part| lib_auv_proteus_hwd.addArg(part);
        }

        const install_lib_auv_proteus_hwd = b.addInstallFileWithDir(lib_auv_proteus_hwd_so, .lib, "libauv_proteus_hwd.so");
        b.getInstallStep().dependOn(&install_lib_auv_proteus_hwd.step);
    }

    const cmake_configure = b.addSystemCommand(&.{
        "cmake",
        "-B",
        "build",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
    });
    if (@hasDecl(std.Build.Step, "addWatchInput")) {
        try cmake_configure.step.addWatchInput(b.path("CMakeLists.txt"));
    }

    const cpu_count = std.Thread.getCpuCount() catch 1;
    const cmake_build = b.addSystemCommand(&.{
        "cmake",
        "--build",
        "build",
        "--parallel",
        b.fmt("{d}", .{cpu_count}),
        "--target",
    });
    cmake_build.step.dependOn(&cmake_configure.step);

    const stonefish = b.option(bool, "stonefish", "Build the Stonefish simulator plugin") orelse false;
    if (stonefish) {
        const cmake_target = "auv_stonefish_sim";
        cmake_build.addArg(cmake_target);

        if (@hasDecl(std.Build.Step, "addDirectoryWatchInput")) {
            _ = try cmake_build.step.addDirectoryWatchInput(b.path("auvs/stonefish_sim"));
        }

        const cmake_dynlib_file = b.fmt("lib{s}{s}", .{ cmake_target, target.result.dynamicLibSuffix() });
        const cmake_dynlib_path = b.path(b.fmt("build/{s}", .{cmake_dynlib_file}));

        const install_stonefish_sim = b.addInstallFileWithDir(cmake_dynlib_path, .lib, cmake_dynlib_file);
        install_stonefish_sim.step.dependOn(&cmake_build.step);

        b.getInstallStep().dependOn(&install_stonefish_sim.step);
    }

    cmake_build.step.dependOn(&cmake_configure.step);
}

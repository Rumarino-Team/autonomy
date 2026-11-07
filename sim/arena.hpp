#pragma once
#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <new>
#include <span>
#include <stdexcept>
#include <sys/mman.h>
#include <type_traits>
#include <unistd.h>

namespace sim {
// A typed bump arena for plain geometry values. One virtual-memory reservation
// backs the entire sequence; the OS supplies physical pages as they are
// touched. Appending never reallocates or moves earlier elements. No PMR
// resource or per-element allocator is involved. The owning Main sets the
// element limit.
template <class T> class ArenaBuffer {
  static_assert(std::is_trivial_v<T> && std::is_standard_layout_v<T>);
  void *mapping_ = nullptr;
  T *data_ = nullptr;
  size_t size_ = 0;
  size_t limit_;
  size_t mapping_bytes_ = 0;
  size_t committed_bytes_ = 0;
  size_t page_size_ = 0;

  void reserve_storage() {
    if (mapping_)
      return;
    const long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0)
      throw std::bad_alloc();
    page_size_ = size_t(page_size);
    const size_t bytes = limit_ * sizeof(T);
    // Keep the original mapping for munmap even for over-aligned element types.
    const size_t padded = bytes + alignof(T) - 1;
    if (padded > std::numeric_limits<size_t>::max() - (page_size_ - 1))
      throw std::bad_alloc();
    const size_t mapped_bytes =
        ((padded + page_size_ - 1) / page_size_) * page_size_;
    // Reserve address space only. allocate() makes just the used page prefix
    // writable, so large index limits do not reserve that much commit memory.
    void *mapping = mmap(nullptr, mapped_bytes, PROT_NONE,
                         MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (mapping == MAP_FAILED)
      throw std::bad_alloc();
    void *aligned = mapping;
    size_t available = mapped_bytes;
    std::align(alignof(T), bytes, aligned, available);
    // Default initialization of trivial values starts the array lifetime but
    // does not initialize its bytes or eagerly touch the reserved pages.
    try {
      data_ = ::new (aligned) T[limit_];
    } catch (...) {
      munmap(mapping, mapped_bytes);
      throw;
    }
    mapping_ = mapping;
    mapping_bytes_ = mapped_bytes;
  }

public:
  explicit ArenaBuffer(size_t max_elements) : limit_(max_elements) {
    if (max_elements >
        (std::numeric_limits<size_t>::max() - (alignof(T) - 1)) / sizeof(T))
      throw std::length_error("typed arena size overflow");
  }
  ~ArenaBuffer() { release(); }
  ArenaBuffer(const ArenaBuffer &) = delete;
  ArenaBuffer &operator=(const ArenaBuffer &) = delete;
  ArenaBuffer(ArenaBuffer &&) = delete;
  ArenaBuffer &operator=(ArenaBuffer &&) = delete;

  size_t size() const { return size_; }
  size_t max_size() const { return limit_; }
  size_t capacity() const { return mapping_ ? limit_ : 0; }
  size_t reserved_bytes() const { return mapping_bytes_; }
  // Writable pages are still backed lazily by the OS; this is not RSS.
  size_t writable_bytes() const { return committed_bytes_; }
  bool empty() const { return size_ == 0; }
  T *data() { return data_; }
  const T *data() const { return data_; }
  T *begin() { return data_; }
  const T *begin() const { return data_; }
  T *end() { return data_ ? data_ + size_ : nullptr; }
  const T *end() const { return data_ ? data_ + size_ : nullptr; }
  T &operator[](size_t i) { return data_[i]; }
  const T &operator[](size_t i) const { return data_[i]; }

  // Allocate a contiguous range by advancing the cursor. The caller fills each
  // returned element before reading it. A failed request leaves size unchanged.
  std::span<T> allocate(size_t count) {
    if (count > limit_ - size_)
      throw std::length_error("typed arena element limit exceeded");
    if (!count)
      return {end(), size_t(0)};
    reserve_storage();
    const size_t offset = reinterpret_cast<std::byte *>(data_) -
                          static_cast<std::byte *>(mapping_);
    const size_t needed = offset + (size_ + count) * sizeof(T);
    const size_t committed =
        ((needed + page_size_ - 1) / page_size_) * page_size_;
    if (committed > committed_bytes_) {
      auto *start = static_cast<std::byte *>(mapping_) + committed_bytes_;
      if (mprotect(start, committed - committed_bytes_,
                   PROT_READ | PROT_WRITE) != 0)
        throw std::bad_alloc();
      committed_bytes_ = committed;
    }
    auto result = std::span<T>(data_ + size_, count);
    size_ += count;
    return result;
  }
  std::span<T> append(std::span<const T> source) {
    auto out = allocate(source.size());
    if (!out.empty())
      std::copy(source.begin(), source.end(), out.begin());
    return out;
  }
  void push_back(const T &value) { append({&value, 1}); }

  // Both operations invalidate previously handed-out logical ranges. reset()
  // retains storage for reuse; release() returns the whole buffer to the OS.
  void reset() noexcept { size_ = 0; }
  void release() noexcept {
    if (mapping_)
      munmap(mapping_, mapping_bytes_);
    mapping_ = nullptr;
    data_ = nullptr;
    size_ = mapping_bytes_ = committed_bytes_ = page_size_ = 0;
  }
};
} // namespace sim

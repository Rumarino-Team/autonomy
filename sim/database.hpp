#pragma once
// Intrusive ownership is independent of allocation. A list owns membership;
// the enclosing Main owns storage. Pointers never leave their owning process.
#include "backend_common.hpp"
#include <iterator>
#include <memory>
#include <memory_resource>
#include <span>
#include <type_traits>
namespace sim {
struct Link {
  Link *next = nullptr, *prev = nullptr;
};
struct ListBase {
  Link *first = nullptr, *last = nullptr;
};
enum class DataKind : uint32_t {
  Object,
  Fault,
  Module,
  MuJoCoBodyBinding,
  MuJoCoShapeBinding,
  StonefishSolidBinding,
  StonefishRigidBinding
};
struct DataBlock {
  Link link{};
  DataKind kind{};
  uint32_t uid = 0;
  const void *owner = nullptr;
  ListBase *list = nullptr;
  uint32_t index = 0;
  // Scene tables may index storage embedded in a fixed Robot. They must not
  // destroy that storage as though it were an independently pooled record.
  bool pooled = true;
};
static_assert(std::is_standard_layout_v<DataBlock> &&
              offsetof(DataBlock, link) == 0);
inline DataBlock &datablock(Link *link) {
  return *reinterpret_cast<DataBlock *>(link);
}
inline void append(ListBase &list, DataBlock &id) {
  require(!id.list && !id.link.next && !id.link.prev,
          "datablock already linked");
  id.list = &list;
  id.link.prev = list.last;
  if (list.last)
    list.last->next = &id.link;
  else
    list.first = &id.link;
  list.last = &id.link;
}
inline void unlink(ListBase &list, DataBlock &id) {
  require(id.list == &list, "datablock belongs to another list");
  if (id.link.prev)
    id.link.prev->next = id.link.next;
  else
    list.first = id.link.next;
  if (id.link.next)
    id.link.next->prev = id.link.prev;
  else
    list.last = id.link.prev;
  id.link = {};
  id.list = nullptr;
}
template <class T>
T &checked_block(DataBlock &id, DataKind kind, const void *owner) {
  static_assert(std::is_standard_layout_v<T> && offsetof(T, id) == 0);
  require(id.kind == kind && id.owner == owner,
          "wrong datablock type or database");
  return *reinterpret_cast<T *>(&id);
}
// All resources are local, never a global allocator override. The injected
// upstream makes failure handling and allocation stabilization measurable.
struct Memory {
  std::pmr::unsynchronized_pool_resource pool;
  std::pmr::monotonic_buffer_resource model;
  std::pmr::monotonic_buffer_resource scratch;
  explicit Memory(
      std::pmr::memory_resource *upstream = std::pmr::get_default_resource(),
      std::pmr::memory_resource *model_upstream = nullptr)
      : pool(upstream), model(model_upstream ? model_upstream : upstream),
        scratch(&pool) {}
  template <class T, class... Args> T *make(Args &&...args) {
    std::pmr::polymorphic_allocator<T> a(&pool);
    T *p = a.allocate(1);
    try {
      std::construct_at(p, std::forward<Args>(args)...);
    } catch (...) {
      a.deallocate(p, 1);
      throw;
    }
    return p;
  }
  template <class T> void destroy(T *p) noexcept {
    std::destroy_at(p);
    std::pmr::polymorphic_allocator<T>(&pool).deallocate(p, 1);
  }
  template <class T> std::span<T> copy_mutable(std::span<const T> source) {
    static_assert(std::is_trivially_copyable_v<T>);
    if (source.empty())
      return {};
    auto *p =
        std::pmr::polymorphic_allocator<T>(&model).allocate(source.size());
    std::uninitialized_copy(source.begin(), source.end(), p);
    return {p, source.size()};
  }
  template <class T> std::span<const T> copy(std::span<const T> source) {
    return copy_mutable(source);
  }
};
// The dense table contains pointers, not copies of records. Iteration remains
// cache-friendly while pool growth cannot invalidate a datablock address.
template <class Block, DataKind Kind> class Records {
  using T = typename Block::Value;
  Memory &memory_;
  const void *owner_;
  const bool &frozen_;
  std::pmr::vector<Block *> dense_;

public:
  ListBase list;
  Records(Memory &memory, const void *owner, const bool &frozen)
      : memory_(memory), owner_(owner), frozen_(frozen), dense_(&memory.pool) {
    static_assert(std::is_standard_layout_v<Block> && offsetof(Block, id) == 0);
  }
  Records(const Records &) = delete;
  ~Records() { clear_unchecked(); }

private:
  void clear_unchecked() noexcept {
    for (auto *p : dense_) {
      unlink(list, p->id);
      if (p->id.pooled)
        memory_.destroy(p);
    }
    dense_.clear();
  }

public:
  void clear() {
    require(!frozen_, "database is frozen");
    clear_unchecked();
  }
  void push_back(const T &value) {
    require(!frozen_, "database is frozen");
    auto *p = memory_.make<Block>();
    p->id = {{}, Kind, uint32_t(dense_.size()), owner_, nullptr};
    p->id.index = uint32_t(dense_.size());
    try {
      p->data = value;
      dense_.push_back(p);
    } catch (...) {
      memory_.destroy(p);
      throw;
    }
    append(list, p->id);
  }
  void borrow(Block &record, const T &value) {
    require(!frozen_, "database is frozen");
    require(!record.id.list, "fixed robot record already registered");
    record.data = value;
    record.id = {{}, Kind, uint32_t(dense_.size()), owner_, nullptr};
    record.id.index = uint32_t(dense_.size());
    record.id.pooled = false;
    dense_.push_back(&record);
    append(list, record.id);
  }
  size_t size() const { return dense_.size(); }
  bool empty() const { return dense_.empty(); }
  Block &block(size_t i) { return *dense_.at(i); }
  const Block &block(size_t i) const { return *dense_.at(i); }
  static Block &block_of(T &value) {
    return *reinterpret_cast<Block *>(reinterpret_cast<std::byte *>(&value) -
                                      offsetof(Block, data));
  }
  static const Block &block_of(const T &value) {
    return block_of(const_cast<T &>(value));
  }
  T &operator[](size_t i) { return block(i).data; }
  const T &operator[](size_t i) const { return block(i).data; }
  T &back() { return (*this)[size() - 1]; }
  const T &back() const { return (*this)[size() - 1]; }
  template <bool Const> class Iter {
    using P = std::conditional_t<Const, Block *const *, Block **>;
    P p_ = nullptr;

  public:
    using iterator_category = std::random_access_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using reference = std::conditional_t<Const, const T &, T &>;
    using pointer = std::conditional_t<Const, const T *, T *>;
    Iter() = default;
    explicit Iter(P p) : p_(p) {}
    reference operator*() const { return (*p_)->data; }
    pointer operator->() const { return &(**this); }
    Iter &operator++() {
      ++p_;
      return *this;
    }
    Iter operator++(int) {
      auto x = *this;
      ++*this;
      return x;
    }
    Iter &operator--() {
      --p_;
      return *this;
    }
    Iter operator--(int) {
      auto x = *this;
      --*this;
      return x;
    }
    Iter &operator+=(difference_type n) {
      p_ += n;
      return *this;
    }
    Iter &operator-=(difference_type n) {
      p_ -= n;
      return *this;
    }
    Iter operator+(difference_type n) const { return Iter(p_ + n); }
    Iter operator-(difference_type n) const { return Iter(p_ - n); }
    difference_type operator-(Iter other) const { return p_ - other.p_; }
    reference operator[](difference_type n) const { return p_[n]->data; }
    auto operator<=>(const Iter &) const = default;
  };
  auto begin() { return Iter<false>(dense_.data()); }
  auto end() { return begin() + size(); }
  auto begin() const { return Iter<true>(dense_.data()); }
  auto end() const { return begin() + size(); }
};
} // namespace sim

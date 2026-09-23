#ifndef FAST_LIVO_LRU_MAP_H_
#define FAST_LIVO_LRU_MAP_H_

#include <cstddef>
#include <iterator>
#include <limits>
#include <list>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

// Owning hash index plus a recency list. The front is most recently used.
// Insertion and touch never evict implicitly: callers prune only after all
// borrowed pointers for the current processing step have been released.
template <typename Key, typename T, typename Hash = std::hash<Key>>
class LruMap
{
public:
  using value_type = std::pair<Key, std::unique_ptr<T>>;
  using list_type = std::list<value_type>;
  using iterator = typename list_type::iterator;
  using const_iterator = typename list_type::const_iterator;

  LruMap() = default;
  LruMap(const LruMap &) = delete;
  LruMap &operator=(const LruMap &) = delete;
  LruMap(LruMap &&) = delete;
  LruMap &operator=(LruMap &&) = delete;

  void configure(bool enabled, std::size_t capacity)
  {
    if (enabled && capacity == 0) throw std::invalid_argument("enabled LRU capacity must be greater than zero");
    enabled_ = enabled;
    capacity_ = capacity;
  }

  bool enabled() const noexcept { return enabled_; }
  std::size_t capacity() const noexcept { return capacity_; }
  std::size_t size() const noexcept { return entries_.size(); }
  bool empty() const noexcept { return entries_.empty(); }
  std::size_t evictionCount() const noexcept { return eviction_count_; }

  iterator begin() noexcept { return entries_.begin(); }
  iterator end() noexcept { return entries_.end(); }
  const_iterator begin() const noexcept { return entries_.begin(); }
  const_iterator end() const noexcept { return entries_.end(); }
  const_iterator cbegin() const noexcept { return entries_.cbegin(); }
  const_iterator cend() const noexcept { return entries_.cend(); }

  iterator find(const Key &key) noexcept
  {
    const auto found = index_.find(key);
    return found == index_.end() ? entries_.end() : found->second;
  }

  const_iterator find(const Key &key) const noexcept
  {
    const auto found = index_.find(key);
    return found == index_.end() ? entries_.end() : const_iterator(found->second);
  }

  bool contains(const Key &key) const noexcept { return index_.find(key) != index_.end(); }

  std::pair<iterator, bool> emplace(const Key &key, std::unique_ptr<T> value)
  {
    const auto existing = index_.find(key);
    if (existing != index_.end()) return {existing->second, false};

    entries_.emplace_front(key, std::move(value));
    try
    {
      const auto indexed = index_.emplace(entries_.front().first, entries_.begin());
      if (!indexed.second)
      {
        entries_.pop_front();
        return {indexed.first->second, false};
      }
    }
    catch (...)
    {
      entries_.pop_front();
      throw;
    }
    return {entries_.begin(), true};
  }

  std::pair<iterator, bool> insertOrAssign(const Key &key, std::unique_ptr<T> value)
  {
    const auto existing = index_.find(key);
    if (existing != index_.end())
    {
      existing->second->second = std::move(value);
      touch(existing->second);
      return {entries_.begin(), false};
    }
    return emplace(key, std::move(value));
  }

  void touch(iterator entry) noexcept
  {
    if (entry == entries_.end() || entry == entries_.begin()) return;
    entries_.splice(entries_.begin(), entries_, entry);
  }

  bool touch(const Key &key) noexcept
  {
    const auto found = index_.find(key);
    if (found == index_.end()) return false;
    touch(found->second);
    return true;
  }

  iterator erase(iterator entry)
  {
    if (entry == entries_.end()) return entry;
    index_.erase(entry->first);
    return entries_.erase(entry);
  }

  bool erase(const Key &key)
  {
    const auto found = index_.find(key);
    if (found == index_.end()) return false;
    entries_.erase(found->second);
    index_.erase(found);
    return true;
  }

  std::size_t prune()
  {
    if (!enabled_) return 0;
    std::size_t removed = 0;
    while (entries_.size() > capacity_)
    {
      auto oldest = std::prev(entries_.end());
      index_.erase(oldest->first);
      entries_.erase(oldest);
      ++removed;
    }
    eviction_count_ += removed;
    return removed;
  }

  void clear() noexcept
  {
    index_.clear();
    entries_.clear();
  }

  const Key *mostRecentKey() const noexcept { return entries_.empty() ? nullptr : &entries_.front().first; }
  const Key *leastRecentKey() const noexcept { return entries_.empty() ? nullptr : &entries_.back().first; }

  bool indexConsistent() const noexcept
  {
    if (index_.size() != entries_.size()) return false;
    for (auto it = entries_.begin(); it != entries_.end(); ++it)
    {
      const auto found = index_.find(it->first);
      if (found == index_.end() || found->second != it) return false;
    }
    return true;
  }

private:
  list_type entries_;
  std::unordered_map<Key, iterator, Hash> index_;
  bool enabled_ = false;
  std::size_t capacity_ = std::numeric_limits<std::size_t>::max();
  std::size_t eviction_count_ = 0;
};

#endif

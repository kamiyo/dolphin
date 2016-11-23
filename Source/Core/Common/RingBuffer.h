// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <algorithm>
#include <atomic>
#include <cstring>
#include <vector>
#include "Common/CommonTypes.h"

// Simple Ring Buffer class.
// Only implemented bulk data writing.
// Add functions as necessary.
// Tail always <= Head
template <class T>
class RingBuffer
{
public:
  RingBuffer() = default;
  explicit RingBuffer(size_t max_size) : m_size(max_size), { m_data.resize(max_size); }
  ~RingBuffer() = default;
  // Resize resets head and tail
  void Resize(size_t max_size)
  {
    m_max_size = max_size;
    m_data.resize(max_size);
    m_head.store(0);
    m_tail.store(0);
  }

  T& operator[](size_t pos) { return m_data[pos % m_max_size]; }
  const T& operator[](size_t pos) const { return m_data[pos % m_max_size]; }
  // will only write up to tail if write size is too large
  void Write(const T* source, size_t length)
  {
    size_t head = m_head.load();
    size_t adjusted = std::min(length + (head - m_tail.load()), m_max_size);
    // calculate if we need wraparound
    signed long long over = adjusted - (m_max_size - (head % m_max_size));

    if (over > 0)
    {
      memcpy(&m_data[head % m_max_size], source, (adjusted - over) * sizeof(T));
      memcpy(&m_data[0], source + (adjusted - over), over * sizeof(T));
    }
    else
    {
      memcpy(&m_data[head % m_max_size], source, adjusted * sizeof(T));
    }

    m_head.fetch_add(adjusted);
  }

  size_t LoadHead() const { return m_head.load(); }
  size_t LoadTail() const { return m_tail.load(); }
  // does nothing if trying to move head before tail
  void StoreHead(const size_t pos)
  {
  size_t tail = m_tail.load();
    if (pos >= tail && (pos - tail) < m_max_size)
      m_head.store(pos);
  }
  // moves tail up, only to as far as head
  void FetchAddTail(const size_t count)
  {
    size_t adjusted = std::min(count, m_head.load() - m_tail.load());
    m_tail.fetch_add(adjusted);
  }
  // does nothing if trying to store tail after head
  void StoreTail(const size_t pos)
  {
    if (pos <= m_head.load())
      m_tail.store(pos);
  }
  size_t MaxSize() const { return m_max_size; }

private:
  std::vector<T> m_data;
  std::atomic<size_t> m_head{0};
  std::atomic<size_t> m_tail{0};

  size_t m_max_size;
};
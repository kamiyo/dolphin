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

template <class T>
class RingBuffer
{
public:
  RingBuffer() = default;
  explicit RingBuffer(size_t size) : m_size(size), m_mask(size - 1) { m_data.resize(size); }
  ~RingBuffer() = default;
  // Resize resets head and tail
  void Resize(size_t size)
  {
    m_size = size;
    m_mask = size - 1;
    m_data.resize(size);
    m_head.store(0);
    m_tail.store(0);
  }

  T& operator[](size_t pos) { return m_data[pos & m_mask]; }
  const T& operator[](size_t pos) const { return m_data[pos & m_mask]; }
  void Write(const T* source, size_t length)
  {
    size_t head = m_head.load();
    // if writing data will cause head to overtake tail, exit
    // don't know if this is best default behavior...
    // maybe write up to tail would be better?
    if (length + ((head - m_tail.load()) & m_mask) >= m_size)
      return;
    // calculate if we need wraparound
    signed long long over = length - (m_size - (head & m_mask));

    if (over > 0)
    {
      memcpy(&m_data[head & m_mask], source, (length - over) * sizeof(T));
      memcpy(&m_data[0], source + (length - over), over * sizeof(T));
    }
    else
    {
      memcpy(&m_data[head & m_mask], source, length * sizeof(T));
    }

    m_head.fetch_add(length);
  }

  size_t LoadHead() const { return m_head.load(); }
  size_t LoadTail() const { return m_tail.load(); }
  void StoreHead(const size_t pos) { m_head.store(pos); }
  void FetchAddTail(const size_t count) { m_tail.fetch_add(count); }
  void StoreTail(const size_t pos) { m_tail.store(pos); }
  size_t Size() const { return m_size; }

private:
  std::vector<T> m_data;
  std::atomic<size_t> m_head{0};
  std::atomic<size_t> m_tail{0};

  size_t m_size;
  size_t m_mask;
};
// Copyright 2014 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <array>
#include <gtest/gtest.h>
#include <thread>

#include "Common/RingBuffer.h"

TEST(RingBuffer, RingBuffer)
{
  //check 0-sized ringbuffer
  RingBuffer<u32> rb;

  EXPECT_EQ(0u, rb.MaxSize());
  EXPECT_EQ(0u, rb.LoadHead());
  EXPECT_EQ(0u, rb.LoadTail());

  // check resize behavior
  rb.Resize(10);

  EXPECT_EQ(10u, rb.MaxSize());
  // should not affect head and tail
  EXPECT_EQ(0u, rb.LoadHead());
  EXPECT_EQ(0u, rb.LoadTail());

  // check random access and circularity
  // writes: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
  // and then writes: [0, 11, 22, 33, 44, 55, 66, 77, 88, 99, 110]
  for (size_t i = 0; i < rb.MaxSize(); i++) {
    rb[i] = static_cast<u32>(i);
    EXPECT_EQ(static_cast<u32>(i), rb[i]);
    EXPECT_EQ(static_cast<u32>(i), rb[i + rb.MaxSize() * i]);
    rb[i + rb.MaxSize() * i] = static_cast<u32>(i + rb.MaxSize() * i);
    EXPECT_EQ(static_cast<u32>(i + rb.MaxSize() * i), rb[i]);
  }
  // should not affect head and tail
  EXPECT_EQ(0u, rb.LoadHead());
  EXPECT_EQ(0u, rb.LoadTail());

  // 0s the array
  // we don't need to reset head and tail because they haven't been moved
  std::fill_n(&rb[0], rb.MaxSize(), 0);

  std::array<u32, 20> ones{ 1 };
  std::array<u32, 20> twos{ 2 };
  std::array<u32, 20> threes{ 3 };

  // write 5 items, check
  // should be: [1, 1, 1, 1, 1, 0, 0, 0, 0, 0]
  //             ^tail          ^ head
  rb.Write(ones.data(), 5);
  EXPECT_EQ(1u, rb[0]);
  EXPECT_EQ(1u, rb[4]);
  EXPECT_EQ(0u, rb[5]);
  EXPECT_EQ(5u, rb.LoadHead());
  EXPECT_EQ(0u, rb.LoadTail());

  // write 10 items, but since tail hasn't moved from 0,
  // should only write 5 items (current head is at 5)
  // should be : [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]
  //              ^tail,head
  rb.Write(twos.data(), 10);
  EXPECT_EQ(10u, rb.LoadHead());
  EXPECT_EQ(2u, rb[rb.LoadHead() - 1]);
  EXPECT_EQ(1u, rb[rb.LoadHead()]);

  // move tail up, test fetchadd and store
  rb.FetchAddTail(5);
  EXPECT_EQ(5u, rb.LoadTail());
  // now: [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]
  //       ^head          ^tail
  rb.StoreTail(10);
  EXPECT_EQ(10u, rb.LoadTail());
  // now: [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]
  //       ^head,tail

  // since tail == head, fetchadd should not change anything
  // because tail cannot be greater than head
  rb.FetchAddTail(25);
  EXPECT_EQ(rb.LoadHead(), rb.LoadTail());

  // move head down; should do nothing here
  rb.StoreHead(0);
  EXPECT_EQ(10u, rb.LoadHead());

  // should work here, causing:
  // [1, 1, 1, 1, 1, 2, 2, 2, 2, 2]
  //  ^tail                ^head
  rb.StoreHead(17);

  // circle write
  // start head  at 17 and tail at 15
  // writing 6 elements should wrap around 3 element
  // head should be at 23
  rb.StoreTail(15);
  rb.Write(threes.data(), 6);
  EXPECT_EQ(3u, rb[0]);
  EXPECT_EQ(3u, rb[2]);
  EXPECT_EQ(3u, rb[7]);
  EXPECT_EQ(23u, rb.LoadHead());
  EXPECT_EQ(1u, rb[rb.LoadHead()]);
  EXPECT_EQ(2u, rb[rb.LoadTail()]);
  // [3, 3, 3, 1, 1, 2, 2, 3, 3, 3]
  //           ^head ^tail
}
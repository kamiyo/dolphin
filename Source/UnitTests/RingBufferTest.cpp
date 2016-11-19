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
  rb[5] = 10;
  EXPECT_EQ(10u, rb[5]);
  EXPECT_EQ(rb[5], rb[15]);
  rb[25] = 1;
  EXPECT_EQ(1u, rb[5]);

  // start here CODING

  std::array<u32, 50000> data{};
  rb1.Write(data.data(), data.size());

  EXPECT_EQ(50000u, rb1.LoadHead());
  EXPECT_EQ(0u, rb1.LoadTail());

  // check circle-write past tail
  // should do nothing
  std::array<u32, 50001> ones;
  std::fill(&ones, &ones + 50001, 1);
  rb1.Write(ones.data(), ones.size());

  EXPECT_EQ(50000u, rb1.LoadHead());
  EXPECT_EQ(0u, rb1[0]);

  // move tail, then check circle-write
  rb1.FetchAddTail(1);
  EXPECT_EQ(1u, rb1.LoadTail());
  rb1.Write(ones.data(), ones.size());

  EXPECT_EQ(100001u, rb1.LoadHead());
  EXPECT_EQ(1u, rb1[0]);

  rb1[0] = 9;
  // check circle-ness
  EXPECT_EQ(9u, rb1[rb1.MaxSize()]);
  EXPECT_EQ(9u, rb1[rb1.MaxSize() * 2]);

  rb1[rb1.MaxSize()] = 5;
  EXPECT_EQ(5u, rb1[0]);
}
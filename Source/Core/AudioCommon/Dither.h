// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <random>

class Dither
{
public:
  Dither();
  ~Dither();
  float GenerateNoise() const;
protected:
  std::mt19937 m_mersenne_twister;
  std::uniform_real_distribution<float> m_real_dist{0, 1};
};

class TriangleDither : public Dither
{
public:
  TriangleDither();
  ~TriangleDither();

private:
  float state_l, state_r;
};
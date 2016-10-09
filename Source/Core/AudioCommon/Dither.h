// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <random>

#include <Common/CommonTypes.h>

class Dither
{
public:
  Dither();
  ~Dither() {}
  void Process(const float* input, s16* output, u32 num_samples);
  // make sure input samples are pre-clamped to [-1, 1]
  virtual void DitherStereoSample(const float* in, s16* out) = 0;
protected:
  float GenerateNoise();
  float ScaleFloatToInt(float in)
  {
    return (in > 0) ? (in * (float)0x7fff) : (in * (float)0x8000);
  }
  std::mt19937 m_mersenne_twister;
  std::uniform_real_distribution<float> m_real_dist{-0.5, 0.5};
};

class TriangleDither : public Dither
{
public:
  TriangleDither() : Dither() {}
  ~TriangleDither() {}
  void DitherStereoSample(const float* in, s16* out);

private:
  float state_l, state_r;
};
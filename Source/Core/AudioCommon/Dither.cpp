// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Dither.h"

#include "Common/MathUtil.h"

Dither::Dither()
{
  m_mersenne_twister.seed(std::random_device{}());
}

float Dither::GenerateNoise()
{
  return m_real_dist(m_mersenne_twister);
}

// num_samples counts LR pair as one sample
void Dither::Process(const float* in, s16* out, u32 num_samples)
{
  for (u32 i = 0; i < num_samples * 2; i += 2)
  {
    DitherStereoSample(&in[i], &out[i]);
  }
}

void TriangleDither::DitherStereoSample(const float* in, s16* out)
{
  float random_l = GenerateNoise();
  float random_r = GenerateNoise();

  float temp_l = ScaleFloatToInt(in[0]) + random_l - state_l;
  float temp_r = ScaleFloatToInt(in[1]) + random_r - state_r;
  out[0] = (s16)MathUtil::Clamp((s32)lrintf(temp_l), -32768, 32767);
  out[1] = (s16)MathUtil::Clamp((s32)lrintf(temp_r), -32768, 32767);
  state_l = random_l;
  state_r = random_r;
}
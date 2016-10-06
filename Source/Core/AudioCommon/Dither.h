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

private:
  std::mt19937 gen_noise;
  float state_l, state_r;
};
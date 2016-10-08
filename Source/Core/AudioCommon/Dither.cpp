// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Dither.h"

Dither::Dither()
{
  m_mersenne_twister.seed(std::random_device{}());
}

Dither::~Dither()
{
}

float Dither::GenerateNoise() const
{
  return m_real_dist(m_mersenne_twister);
}

TriangleDither::TriangleDither()
{
}

TriangleDither::~TriangleDither()
{
}
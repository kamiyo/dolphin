// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <string>
#include "Common/Logging/Log.h"

#include "Common/CommonTypes.h"

/*
Creates kaiser-windowed ideal lowpass filter

Tables are stored transposed num_crossings x samples_per_crossing
instead of samples_per_crossing x num_crossings
*/
class WindowedSincFilter
{
public:
  // Default Parameters changeable
  WindowedSincFilter(u32 num_crossings = 17, u32 samples_per_crossing = 512, float cutoff_cycle = 0.5, float beta = 7.0)
    : m_num_crossings(num_crossings)
    , m_samples_per_crossing(samples_per_crossing)
    , m_wing_size(samples_per_crossing * (num_crossings - 1) / 2)
    , m_cutoff_cycle(cutoff_cycle)
    , m_kaiser_beta(beta)
  {
    m_coeffs = (float*)malloc(m_wing_size * sizeof(float));
    m_deltas = (float*)malloc(m_wing_size * sizeof(float));
    CheckFilterCache();
  }

  ~WindowedSincFilter()
  {
    free(m_coeffs);
    free(m_deltas);
    m_coeffs = m_deltas = nullptr;
  }

  u32 GetColumnIndex(const u32 frac_index) const
  {
    return frac_index * (m_num_crossings - 1) / 2;
  }

  const u32 m_num_crossings;        // npc
  const u32 m_samples_per_crossing; // spc
  const u32 m_wing_size;            // spc * (nc - 1) / 2

  float* m_coeffs;
  float* m_deltas;

private:
  void CheckFilterCache();
  void PopulateFilterCoeffs();
  void PopulateFilterDeltas();
  void TransposeTables();
  bool ReadFilterFromFile(const std::string &filename, float* filter);
  bool WriteFilterToFile(const std::string &filename, float* filter);
  double ModBesselZeroth(const double x) const;

  const float m_kaiser_beta;        // parameter for Kaiser window
  const float m_cutoff_cycle;       // cutoff frequency (ratio of nyquist frequency)

};


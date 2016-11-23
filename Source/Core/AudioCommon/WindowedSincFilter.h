// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <vector>

#include "AudioCommon/BaseFilter.h"
#include "Common/CommonTypes.h"
// Creates kaiser-windowed ideal lowpass filter
// Generates and stores only one side of the impulse response as it is symmetrical.
// Thus, when performing convolution, first apply the filter as is stored to samples
// before current time, and then to samples after.
//
// Quality of filter = sharpness of rolloff + attenuation of stopband
//
// num_crossings: affects transition-band-width, the sharper the rolloff, the more taps needed.
//				also slightly affects stopband ripple magnitude.
//				Quality:
//				   low:    < 17
//				   medium: between 17 and 35
//				   high:   > 35
//
// samples_per_crossing: affects the accuracy of the filter response; this is dependent
//                       on the bit-length of each filter sample. samples_per_crossing
//                       should be around 2 ^ (sample_bit_length / 2). In this case,
//                       since float = 32bit, we choose 512. We are not really memory limited
//                       at this magnitude, so we don't need to change this parameter.
//
// beta: parameter for Kaiser window (tradeoff between main-lobe width and side-lobes levels).
//       This affects the stopband ripple attenuation: 8.0 = about 70db attenuation
//
// see ccrma.standford.edu/~jos/resample/
// CCRMA uses 27 taps x 512 samples for high quality.
//
// cutoff_cycle: the center of the transition band (0.5 gives samperate / 2).
//
// All combined, these default parameters give a filter with -80db stopband attenuation and a transition
// width of about 0.5 the nyquist frequency centered on the nyquist frequency

class WindowedSincFilter final : public BaseFilter
{
public:
  WindowedSincFilter(u32 num_crossings = 21, u32 samples_per_crossing = 512,
                     float cutoff_cycle = 0.5, float beta = 8.0);

  ~WindowedSincFilter() = default;
  void ConvolveStereo(const RingBuffer<float>& input, u32 index, float* output_l, float* output_r,
                      float fraction, float ratio) const override;

private:
  static constexpr double BESSEL_EPSILON = 1e-21;
  static constexpr double PI = 3.14159265358979323846;

  void PopulateFilterCoeffs();
  void PopulateFilterDeltas();
  double ModBesselZeroth(const double x) const;

  // upsampling is a bit simpler than downsampling so clearer to split into two functions
  void UpSampleStereo(const RingBuffer<float>& input, u32 index, float* output_l, float* output_r,
                      const float fraction) const;
  void DownSampleStereo(const RingBuffer<float>& input, u32 index, float* output_l, float* output_r,
                        const float fraction, const float ratio) const;

  // cutoff frequency (ratio of samplerate)
  const float m_cutoff_cycle;
  const float m_kaiser_beta;

  const u32 m_num_crossings;         // filter taps
  const u32 m_samples_per_crossing;  // filter resolution
  const u32 m_wing_size;             // one-sided length of filter

  std::vector<float> m_coeffs;
  std::vector<float> m_deltas;  // store deltas for interpolating between filter values
};
// Copyright 2009 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include "WindowedSincFilter.h"

#include <array>
#include <atomic>
#include <iterator>

#include "AudioCommon/WaveFile.h"
#include "Common/CommonTypes.h"
#include "Dither.h"

class CMixer final
{
public:
  explicit CMixer(u32 BackendSampleRate);
  ~CMixer();

  // Called from audio threads
  u32 Mix(s16* samples, u32 numSamples, bool consider_framelimit = true);

  // Called from main thread
  void PushSamples(const s16* samples, u32 num_samples);
  void PushStreamingSamples(const s16* samples, u32 num_samples);
  void PushWiimoteSpeakerSamples(const s16* samples, u32 num_samples, u32 sample_rate);
  u32 GetSampleRate() const { return m_sampleRate; }
  void SetDMAInputSampleRate(u32 rate);
  void SetStreamInputSampleRate(u32 rate);
  void SetStreamingVolume(u32 lvolume, u32 rvolume);
  void SetWiimoteSpeakerVolume(u32 lvolume, u32 rvolume);

  void StartLogDTKAudio(const std::string& filename);
  void StopLogDTKAudio();

  void StartLogDSPAudio(const std::string& filename);
  void StopLogDSPAudio();

  float GetCurrentSpeed() const { return m_speed.load(); }
  void UpdateSpeed(float val) { m_speed.store(val); }

  static inline float lerp(float sample1, float sample2, float fraction) {
      return (1.f - fraction) * sample1 + fraction * sample2;
  }

private:
  static constexpr u32 MAX_SAMPLES = 1024 * 4;  // 128 ms
  static constexpr u32 INDEX_MASK = MAX_SAMPLES * 2 - 1;
  static constexpr int MAX_FREQ_SHIFT = 200;  // Per 32000 Hz
  static constexpr float CONTROL_FACTOR = 0.2f;
  static constexpr u32 CONTROL_AVG = 32;  // In freq_shift per FIFO size offset

  class MixerFifo
  {
  public:
    MixerFifo(CMixer* mixer, unsigned sample_rate, std::shared_ptr<WindowedSincFilter> filter = nullptr)
        : m_mixer(mixer)
        , m_input_sample_rate(sample_rate)
        , m_filter(filter)
        , m_filter_length((filter) ? ((filter->m_num_crossings - 1) / 2) : 1)
    {}
    virtual void Interpolate(u32 index, float* output_l, float* output_r) = 0;
    void PushSamples(const s16* samples, u32 num_samples);
    u32 Mix(std::array<float, MAX_SAMPLES * 2>& samples, u32 numSamples, bool consider_framelimit = true);
    void SetInputSampleRate(u32 rate);
    u32 GetInputSampleRate() const;
    void SetVolume(u32 lvolume, u32 rvolume);

  protected:
    std::array<float, MAX_SAMPLES * 2> m_floats{};
    float m_numLeftI = 0.0f;
    float m_frac = 0;
    std::shared_ptr<WindowedSincFilter> m_filter;
    u32 m_filter_length;
    u32 timer = 0;

  private:
    CMixer* m_mixer;
    unsigned m_input_sample_rate;
    std::array<s16, MAX_SAMPLES * 2> m_buffer{};
    std::atomic<u32> m_indexW{0};
    std::atomic<u32> m_indexR{0};
    std::atomic<u32> m_floatI{0};
    // Volume ranges from 0-256
    std::atomic<s32> m_LVolume{256};
    std::atomic<s32> m_RVolume{256};
  };

  class LinearMixer : public MixerFifo
  {
  public:
    LinearMixer(CMixer* mixer, u32 sample_rate)
      : MixerFifo(mixer, sample_rate)
    {}
    void Interpolate(u32 index, float* output_l, float* output_r);

  };

  class SincMixer : public MixerFifo
  {
  public:
    SincMixer(CMixer* mixer, u32 sample_rate, std::shared_ptr<WindowedSincFilter> filter)
      : MixerFifo(mixer, sample_rate, filter)
    {}
    void Interpolate(u32 index, float* output_l, float* output_r);

  };

  std::array<float, MAX_SAMPLES * 2> m_accumulator{};

  std::unique_ptr<MixerFifo> m_dma_mixer;
  std::unique_ptr<MixerFifo> m_streaming_mixer;
  std::unique_ptr<MixerFifo> m_wiimote_speaker_mixer;
  std::unique_ptr<Dither> m_dither;
  u32 m_sampleRate;

  WaveFileWriter m_wave_writer_dtk;
  WaveFileWriter m_wave_writer_dsp;
  WaveFileWriter m_wave_writer_debug;

  bool m_log_dtk_audio = false;
  bool m_log_dsp_audio = false;

  // Current rate of emulation (1.0 = 100% speed)
  std::atomic<float> m_speed{0.0f};

};

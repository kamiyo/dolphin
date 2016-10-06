// Copyright 2008 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include <algorithm>
#include <cstring>

#include "AudioCommon/AudioCommon.h"
#include "AudioCommon/Mixer.h"
#include "Common/CommonFuncs.h"
#include "Common/CommonTypes.h"
#include "Common/Logging/Log.h"
#include "Common/MathUtil.h"
#include "Core/ConfigManager.h"

CMixer::CMixer(u32 BackendSampleRate) : m_sampleRate(BackendSampleRate)
{
  m_wiimote_speaker_mixer = std::make_unique<LinearMixer>(this, 3000);

  std::shared_ptr<WindowedSincFilter> FIR_filter(new WindowedSincFilter());

  m_dma_mixer = std::make_unique<SincMixer>(this, 32000, FIR_filter);
  m_streaming_mixer = std::make_unique<SincMixer>(this, 48000, FIR_filter);

  INFO_LOG(AUDIO_INTERFACE, "Mixer is initialized");
}

CMixer::~CMixer()
{
}

// Executed from sound stream thread
u32 CMixer::MixerFifo::Mix(std::array<float, MAX_SAMPLES * 2>& samples, u32 numSamples, bool consider_framelimit = true)
{
  u32 currentSample = 0;

  // Cache access in non-volatile variable
  // This is the only function changing the read value, so it's safe to
  // cache it locally although it's written here.
  // The writing pointer will be modified outside, but it will only increase,
  // so we will just ignore new written data while interpolating.
  // Without this cache, the compiler wouldn't be allowed to optimize the
  // interpolation loop.
  u32 indexR = m_indexR.load();
  u32 indexW = m_indexW.load();

  u32 low_waterwark = m_input_sample_rate * SConfig::GetInstance().iTimingVariance / 1000;
  low_waterwark = std::min(low_waterwark, MAX_SAMPLES / 2);

  float numLeft = (float)(((indexW - indexR) & INDEX_MASK) / 2);
  m_numLeftI = (numLeft + m_numLeftI * (CONTROL_AVG - 1)) / CONTROL_AVG;
  float offset = (m_numLeftI - low_waterwark) * CONTROL_FACTOR;
  if (offset > MAX_FREQ_SHIFT)
    offset = MAX_FREQ_SHIFT;
  if (offset < -MAX_FREQ_SHIFT)
    offset = -MAX_FREQ_SHIFT;

  // render numleft sample pairs to samples[]
  // advance indexR with sample position
  // remember fractional offset

  float emulationspeed = SConfig::GetInstance().m_EmulationSpeed;
  float aid_sample_rate = m_input_sample_rate + offset;
  if (consider_framelimit && emulationspeed > 0.0f)
  {
    aid_sample_rate = aid_sample_rate * emulationspeed;
  }

  const float ratio = aid_sample_rate / (float)m_mixer->m_sampleRate;

  float lvolume = m_LVolume.load();
  float rvolume = m_RVolume.load();
  u32 floatI = m_floatI.load();
  for (; ((indexW - floatI) & INDEX_MASK) > 0; ++floatI) {
    m_floats[floatI & INDEX_MASK] = MathUtil::SignedShortToFloat(Common::swap16(m_buffer[floatI & INDEX_MASK]));

  }

  m_floatI.store(floatI);

  for (; currentSample < numSamples * 2 && ((indexW - indexR) & INDEX_MASK) >(m_filter_length + 5) * 2; currentSample += 2)
  {
    float sample_l, sample_r;
    Interpolate(indexR, &sample_l, &sample_r);

    samples[currentSample + 1] += sample_l * lvolume;
    samples[currentSample] += sample_r * rvolume;

    m_frac += ratio;
    indexR += 2 * (s32)m_frac;
    m_frac = m_frac - (s32)m_frac;

  }


  // Padding
  float s[2];
  s[0] = m_floats[(indexR - 1) & INDEX_MASK] * rvolume;
  s[1] = m_floats[(indexR - 2) & INDEX_MASK] * lvolume;

  for (; currentSample < numSamples * 2; currentSample += 2)
  {
    samples[currentSample] += s[0];
    samples[currentSample + 1] += s[1];
  }

  // Flush cached variable
  m_indexR.store(indexR);

  return numSamples;
}

u32 CMixer::Mix(s16* samples, u32 num_samples, bool consider_framelimit)
{
  if (!samples)
    return 0;

  memset(samples, 0, num_samples * 2 * sizeof(s16));
  std::fill_n(m_accumulator.begin(), num_samples * 2, 0);

  m_dma_mixer->Mix(m_accumulator, num_samples, consider_framelimit);
  m_streaming_mixer->Mix(m_accumulator, num_samples, consider_framelimit);
  m_wiimote_speaker_mixer->Mix(m_accumulator, num_samples, consider_framelimit);

  //dither here

  return num_samples;
}

void CMixer::MixerFifo::PushSamples(const s16* samples, u32 num_samples)
{
  // Cache access in non-volatile variable
  // indexR isn't allowed to cache in the audio throttling loop as it
  // needs to get updates to not deadlock.
  u32 indexW = m_indexW.load();

  // Check if we have enough free space
  // indexW == m_indexR results in empty buffer, so indexR must always be smaller than indexW
  if (num_samples * 2 + ((indexW - m_indexR.load()) & INDEX_MASK) >= MAX_SAMPLES * 2)
    return;

  // AyuanX: Actual re-sampling work has been moved to sound thread
  // to alleviate the workload on main thread
  // and we simply store raw data here to make fast mem copy
  int over_bytes = num_samples * 4 - (MAX_SAMPLES * 2 - (indexW & INDEX_MASK)) * sizeof(s16);
  if (over_bytes > 0)
  {
    memcpy(&m_buffer[indexW & INDEX_MASK], samples, num_samples * 4 - over_bytes);
    memcpy(&m_buffer[0], samples + (num_samples * 4 - over_bytes) / sizeof(s16), over_bytes);
  }
  else
  {
    memcpy(&m_buffer[indexW & INDEX_MASK], samples, num_samples * 4);
  }

  m_indexW.fetch_add(num_samples * 2);
}

void CMixer::PushSamples(const s16* samples, u32 num_samples)
{
  m_dma_mixer->PushSamples(samples, num_samples);
  int sample_rate = m_dma_mixer->GetInputSampleRate();
  if (m_log_dsp_audio)
    m_wave_writer_dsp.AddStereoSamplesBE(samples, num_samples, sample_rate);
}

void CMixer::PushStreamingSamples(const s16* samples, u32 num_samples)
{
  m_streaming_mixer->PushSamples(samples, num_samples);
  int sample_rate = m_streaming_mixer->GetInputSampleRate();
  if (m_log_dtk_audio)
    m_wave_writer_dtk.AddStereoSamplesBE(samples, num_samples, sample_rate);
}

void CMixer::PushWiimoteSpeakerSamples(const s16* samples, u32 num_samples,
                                       u32 sample_rate)
{
  s16 samples_stereo[MAX_SAMPLES * 2];

  if (num_samples < MAX_SAMPLES)
  {
    m_wiimote_speaker_mixer->SetInputSampleRate(sample_rate);

    for (u32 i = 0; i < num_samples; ++i)
    {
      samples_stereo[i * 2] = Common::swap16(samples[i]);
      samples_stereo[i * 2 + 1] = Common::swap16(samples[i]);
    }

    m_wiimote_speaker_mixer->PushSamples(samples_stereo, num_samples);
  }
}

void CMixer::SetDMAInputSampleRate(u32 rate)
{
  m_dma_mixer->SetInputSampleRate(rate);
}

void CMixer::SetStreamInputSampleRate(u32 rate)
{
  m_streaming_mixer->SetInputSampleRate(rate);
}

void CMixer::SetStreamingVolume(u32 lvolume, u32 rvolume)
{
  m_streaming_mixer->SetVolume(lvolume, rvolume);
}

void CMixer::SetWiimoteSpeakerVolume(u32 lvolume, u32 rvolume)
{
  m_wiimote_speaker_mixer->SetVolume(lvolume, rvolume);
}

void CMixer::StartLogDTKAudio(const std::string& filename)
{
  if (!m_log_dtk_audio)
  {
    m_log_dtk_audio = true;
    m_wave_writer_dtk.Start(filename, m_streaming_mixer->GetInputSampleRate());
    m_wave_writer_dtk.SetSkipSilence(false);
    NOTICE_LOG(AUDIO, "Starting DTK Audio logging");
  }
  else
  {
    WARN_LOG(AUDIO, "DTK Audio logging has already been started");
  }
}

void CMixer::StopLogDTKAudio()
{
  if (m_log_dtk_audio)
  {
    m_log_dtk_audio = false;
    m_wave_writer_dtk.Stop();
    NOTICE_LOG(AUDIO, "Stopping DTK Audio logging");
  }
  else
  {
    WARN_LOG(AUDIO, "DTK Audio logging has already been stopped");
  }
}

void CMixer::StartLogDSPAudio(const std::string& filename)
{
  if (!m_log_dsp_audio)
  {
    m_log_dsp_audio = true;
    m_wave_writer_dsp.Start(filename, m_dma_mixer->GetInputSampleRate());
    m_wave_writer_dsp.SetSkipSilence(false);
    NOTICE_LOG(AUDIO, "Starting DSP Audio logging");
  }
  else
  {
    WARN_LOG(AUDIO, "DSP Audio logging has already been started");
  }
}

void CMixer::StopLogDSPAudio()
{
  if (m_log_dsp_audio)
  {
    m_log_dsp_audio = false;
    m_wave_writer_dsp.Stop();
    NOTICE_LOG(AUDIO, "Stopping DSP Audio logging");
  }
  else
  {
    WARN_LOG(AUDIO, "DSP Audio logging has already been stopped");
  }
}

void CMixer::MixerFifo::SetInputSampleRate(u32 rate)
{
  m_input_sample_rate = rate;
}

u32 CMixer::MixerFifo::GetInputSampleRate() const
{
  return m_input_sample_rate;
}

void CMixer::MixerFifo::SetVolume(u32 lvolume, u32 rvolume)
{
  m_LVolume.store(lvolume);
  m_RVolume.store(rvolume);
}

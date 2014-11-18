// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2
// Refer to the license.txt file included.

#include "AudioCommon/AudioCommon.h"
#include "AudioCommon/Mixer.h"
#include "Common/Atomic.h"
#include "Common/CPUDetect.h"
#include "Common/MathUtil.h"
#include "Core/ConfigManager.h"
#include "Core/Core.h"
#include "Core/HW/AudioInterface.h"
#include "Core/HW/VideoInterface.h"

// UGLINESS
#include "Core/PowerPC/PowerPC.h"

#if _M_SSE >= 0x301 && !(defined __GNUC__ && !defined __SSSE3__)
#include <tmmintrin.h>
#endif

#define _USE_MATH_DEFINES
#include <math.h>


inline float Signed16ToFloat(s16 s)
{
	return (s > 0) ? (float) (s / (float) 0x7fff) : (float) (s / (float) 0x8000);
}

inline s16 FloatToSigned16(float f)
{
	return (f > 0) ? (s16) (f * 0x7fff) : (s16) (f * 0x8000);
}

inline float SincSinc(float x, float window_width)
{
	float pi_x = M_PI * x;
	return (float) (window_width * sin(pi_x) * sin(pi_x / window_width) / (pi_x * pi_x));
}

void CMixer::MixerFifo::PopulateFloats(u32 start, u32 stop)
{
	for (u32 i = start; i < stop; ++i)
	{
		m_float_buffer[i & INDEX_MASK] = Signed16ToFloat(Common::swap16(m_buffer[i & INDEX_MASK]));
	}
}

void CMixer::MixerFifo::PopulateSincTable() {
	float table_center = SINC_SIZE / 2;
	for (int i = 0; i < SINC_FSIZE; ++i)
	{
		float offset = 1 - table_center - (float) i / SINC_FSIZE;
		for (int j = 0; j < SINC_SIZE; ++j) {
			float x = j + offset;
			if (x < -2 || x > 2) {
				m_sinc_table[i][j] = 0;
			}
			else if (x == 0) {
				m_sinc_table[i][j] = 1;
			}
			else {
				m_sinc_table[i][j] = SincSinc(x, table_center);
			}
		}
	}
}
// Executed from sound stream thread
u32 CMixer::MixerFifo::Mix(s16* samples, u32 numSamples, bool consider_framelimit)
{
	u32 current_sample = 0;

	// Cache access in non-volatile variable
	// This is the only function changing the read value, so it's safe to
	// cache it locally although it's written here.
	// The writing pointer will be modified outside, but it will only increase,
	// so we will just ignore new written data while interpolating.
	// Without this cache, the compiler wouldn't be allowed to optimize the
	// interpolation loop.
	u32 read_index = Common::AtomicLoad(m_read_index);
	u32 write_index = Common::AtomicLoad(m_write_index);

	float num_left = (float) (((write_index - read_index) & INDEX_MASK) / 2);
	m_num_left_i = (num_left + m_num_left_i*(CONTROL_AVG - 1)) / CONTROL_AVG;
	float offset = (m_num_left_i - LOW_WATERMARK) * CONTROL_FACTOR;
	if (offset > MAX_FREQ_SHIFT) offset = MAX_FREQ_SHIFT;
	if (offset < -MAX_FREQ_SHIFT) offset = -MAX_FREQ_SHIFT;

	//render numleft sample pairs to samples[]
	//advance indexR with sample position
	//remember fractional offset

	u32 framelimit = SConfig::GetInstance().m_Framelimit;
	float aid_sample_rate = m_input_sample_rate + offset;
	if (consider_framelimit && framelimit > 1)
	{
		aid_sample_rate = aid_sample_rate * (framelimit - 1) * 5 / VideoInterface::TargetRefreshRate;
	}

	float ratio = aid_sample_rate / (float) m_mixer->m_sample_rate;

	float lvolume = (float) m_LVolume / 256.f;
	float rvolume = (float) m_RVolume / 256.f;
	s32 lIvolume = m_LVolume;
	s32 rIvolume = m_RVolume;

	m_errorL1 = 0, m_errorL2 = 0;
	m_errorR1 = 0, m_errorR2 = 0;
	m_randR1 = 0, m_randR2 = 0, m_randR1 = 0, m_randR2 = 0;
	float templ, tempr;

	for (; current_sample < numSamples * 2 && ((write_index - read_index) & INDEX_MASK) > 2; current_sample += 2) {
		// get sinc table with *closest* desired offset
		s32 index = (s32) (m_fraction * SINC_FSIZE);
		const float* table0 = m_sinc_table[index];

		u32 read_index_previous = read_index - 2;   // sample -1
		                                            // sample  0 (read_index)
		u32 read_index_next = read_index + 2;       // sample  1
		u32 read_index_nextnext = read_index + 4;   // sample  2

		// LEFT CHANNEL
		float sl1 = m_float_buffer[(read_index_previous) & INDEX_MASK];
		float sl2 = m_float_buffer[(read_index)          & INDEX_MASK];
		float sl3 = m_float_buffer[(read_index_next)     & INDEX_MASK];
		float sl4 = m_float_buffer[(read_index_nextnext) & INDEX_MASK];
		
		float al = sl1 * table0[0];
		float bl = sl2 * table0[1];
		float cl = sl3 * table0[2];
		float dl = sl4 * table0[3];
		float sampleL = al + bl + cl + dl;
		
		//float sampleL = l[0];
		sampleL = sampleL * lvolume;
		sampleL += Signed16ToFloat(samples[current_sample + 1]);

		// dither
		m_randL2 = m_randL1;
		m_randL1 = rand();
		templ = sampleL + DITHER_SHAPE * (m_errorL1 + m_errorL1 - m_errorL2);
		sampleL = templ + DITHER_OFFSET + DITHER_SIZE * (float) (m_randL1 - m_randL2);

		// clamp and output
		MathUtil::Clamp(&sampleL, -1.f, 1.f);
		int sampleLi = FloatToSigned16(sampleL);
		samples[current_sample + 1] = sampleLi;

		// update dither accumulators
		m_errorL2 = m_errorL1;
		m_errorL1 = templ - sampleL;

		// RIGHT CHANNEL
		float sr1 = m_float_buffer[(read_index_previous + 1) & INDEX_MASK];
		float sr2 = m_float_buffer[(read_index          + 1) & INDEX_MASK];
		float sr3 = m_float_buffer[(read_index_next     + 1) & INDEX_MASK];
		float sr4 = m_float_buffer[(read_index_nextnext + 1) & INDEX_MASK];
		
		float ar = sr1 * table0[0];
		float br = sr2 * table0[1];
		float cr = sr3 * table0[2];
		float dr = sr4 * table0[3];
		float sampleR = ar + br + cr + dr;
		

		//float sampleR = r[0];
		sampleR = sampleR * rvolume;
		sampleR += Signed16ToFloat(samples[current_sample]);

		m_randR2 = m_randR1;
		m_randR1 = rand();
		tempr = sampleR + DITHER_SHAPE * (m_errorR1 + m_errorR1 - m_errorR2);
		sampleR = tempr + DITHER_OFFSET + DITHER_SIZE * (float) (m_randR1 - m_randR2);

		MathUtil::Clamp(&sampleR, -1.f, 1.f);
		int sampleRi = FloatToSigned16(sampleR);
		samples[current_sample] = sampleRi;

		m_errorR2 = m_errorR1;
		m_errorR1 = tempr - sampleR;

		m_fraction += ratio;
		read_index += 2 * (int) m_fraction;
		m_fraction = m_fraction - (int) m_fraction;
	}

	// Padding
	s16 s[2];
	s[0] = Common::swap16(m_buffer[(read_index - 1) & INDEX_MASK]);
	s[1] = Common::swap16(m_buffer[(read_index - 2) & INDEX_MASK]);
	s[0] = (s[0] * rIvolume) >> 8;
	s[1] = (s[1] * lIvolume) >> 8;
	for (; current_sample < numSamples * 2; current_sample += 2)
	{
		int sampleR = s[0] + samples[current_sample];
		MathUtil::Clamp(&sampleR, -32768, 32767);
		samples[current_sample] = sampleR;
		int sampleL = s[1] + samples[current_sample + 1];
		MathUtil::Clamp(&sampleL, -32768, 32767);
		samples[current_sample + 1] = sampleL;
	}

	// Flush cached variable
	Common::AtomicStore(m_read_index, read_index);

	return numSamples;
}

u32 CMixer::Mix(s16* samples, u32 num_samples, bool consider_framelimit)
{
	if (!samples)
		return 0;

	std::lock_guard<std::mutex> lk(m_cs_mixing);

	memset(samples, 0, num_samples * 2 * sizeof(s16));

	if (PowerPC::GetState() != PowerPC::CPU_RUNNING)
	{
		// Silence
		return num_samples;
	}

	m_dma_mixer.Mix(samples, num_samples, consider_framelimit);
	m_streaming_mixer.Mix(samples, num_samples, consider_framelimit);
	m_wiimote_speaker_mixer.Mix(samples, num_samples, consider_framelimit);
	return num_samples;
}

void CMixer::MixerFifo::PushSamples(const s16 *samples, u32 num_samples)
{
	// Cache access in non-volatile variable
	// indexR isn't allowed to cache in the audio throttling loop as it
	// needs to get updates to not deadlock.
	u32 current_write_index = Common::AtomicLoad(m_write_index);
	u32 previous_index_write = current_write_index;

	// Check if we have enough free space
	// indexW == m_indexR results in empty buffer, so indexR must always be smaller than indexW
	if (num_samples * 2 + ((current_write_index - Common::AtomicLoad(m_read_index)) & INDEX_MASK) >= MAX_SAMPLES * 2)
		return;

	// AyuanX: Actual re-sampling work has been moved to sound thread
	// to alleviate the workload on main thread
	// and we simply store raw data here to make fast mem copy
	int over_bytes = num_samples * 4 - (MAX_SAMPLES * 2 - (current_write_index & INDEX_MASK)) * sizeof(s16);
	if (over_bytes > 0)
	{
		memcpy(&m_buffer[current_write_index & INDEX_MASK], samples, num_samples * 4 - over_bytes);
		memcpy(&m_buffer[0], samples + (num_samples * 4 - over_bytes) / sizeof(s16), over_bytes);
	}
	else
	{
		memcpy(&m_buffer[current_write_index & INDEX_MASK], samples, num_samples * 4);
	}

	Common::AtomicAdd(m_write_index, num_samples * 2);

	current_write_index = Common::AtomicLoad(m_write_index);
	PopulateFloats(previous_index_write, current_write_index);

	return;
}

void CMixer::PushSamples(const s16 *samples, u32 num_samples)
{
	m_dma_mixer.PushSamples(samples, num_samples);
	if (m_log_dsp_audio)
		g_wave_writer_dsp.AddStereoSamplesBE(samples, num_samples);
}

void CMixer::PushStreamingSamples(const s16 *samples, u32 num_samples)
{
	m_streaming_mixer.PushSamples(samples, num_samples);
	if (m_log_dtk_audio)
		g_wave_writer_dtk.AddStereoSamplesBE(samples, num_samples);
}

void CMixer::PushWiimoteSpeakerSamples(const s16 *samples, u32 num_samples, u32 sample_rate)
{
	s16 samples_stereo[MAX_SAMPLES * 2];

	if (num_samples < MAX_SAMPLES)
	{
		m_wiimote_speaker_mixer.SetInputSampleRate(sample_rate);

		for (u32 i = 0; i < num_samples; ++i)
		{
			samples_stereo[i * 2] = Common::swap16(samples[i]);
			samples_stereo[i * 2 + 1] = Common::swap16(samples[i]);
		}

		m_wiimote_speaker_mixer.PushSamples(samples_stereo, num_samples);
	}
}

void CMixer::SetDMAInputSampleRate(u32 rate)
{
	m_dma_mixer.SetInputSampleRate(rate);
}

void CMixer::SetStreamInputSampleRate(u32 rate)
{
	m_streaming_mixer.SetInputSampleRate(rate);
}

void CMixer::SetStreamingVolume(u32 lvolume, u32 rvolume)
{
	m_streaming_mixer.SetVolume(lvolume, rvolume);
}

void CMixer::SetWiimoteSpeakerVolume(u32 lvolume, u32 rvolume)
{
	m_wiimote_speaker_mixer.SetVolume(lvolume, rvolume);
}

void CMixer::MixerFifo::SetInputSampleRate(u32 rate)
{
	m_input_sample_rate = rate;
}

void CMixer::MixerFifo::SetVolume(u32 lvolume, u32 rvolume)
{
	m_LVolume = lvolume + (lvolume >> 7);
	m_RVolume = rvolume + (rvolume >> 7);
}

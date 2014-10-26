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

float CMixer::MixerFifo::twos2float(u16 s) {
	int n = (s16) s;
	if (n > 0) {
		return (float) (n / (float) 0x7fff);
	}
	else {
		return (float) (n / (float) 0x8000);
	}
}

// converts float to s16
s16  CMixer::MixerFifo::float2stwos(float f) {
	int n;
	if (f > 0) {
		n = (s16) (f * 0x7fff);
	}
	else {
		n = (s16) (f * 0x8000);
	}
	return (s16) n;
}

float CMixer::MixerFifo::sinc_sinc(float x, float window_width) {
	return (float) (window_width * sin(M_PI * x) * sin(M_PI * x / window_width) / ((M_PI * x) * (M_PI * x)));
}

void CMixer::MixerFifo::populateFloats(u32 start, u32 stop) {
	for (u32 i = start; i < stop; i++)
	{
		float_buffer[i & INDEX_MASK] = twos2float(Common::swap16(m_buffer[i & INDEX_MASK]));
	}
}

void CMixer::MixerFifo::populate_sinc_table() {
	float center = SINC_SIZE / 2;
	for (int i = 0; i < SINC_FSIZE; i++) {
		float offset = -1.f * (float) i / (float) SINC_FSIZE;
		for (int j = 0; j < SINC_SIZE; j++) {
			float x = (j + 1 + offset - center);
			if (x < -2 || x > 2) {
				m_sinc_table[i][j] = 0;
			}
			else if (x == 0) {
				m_sinc_table[i][j] = 1;
			}
			else {
				m_sinc_table[i][j] = sinc_sinc(x, center);
			}
		}
	}
}
// Executed from sound stream thread
unsigned int CMixer::MixerFifo::Mix(short* samples, unsigned int numSamples, bool consider_framelimit)
{
	unsigned int currentSample = 0;

	// Cache access in non-volatile variable
	// This is the only function changing the read value, so it's safe to
	// cache it locally although it's written here.
	// The writing pointer will be modified outside, but it will only increase,
	// so we will just ignore new written data while interpolating.
	// Without this cache, the compiler wouldn't be allowed to optimize the
	// interpolation loop.
	u32 indexR = Common::AtomicLoad(m_indexR);
	u32 indexW = Common::AtomicLoad(m_indexW);

	float numLeft = (float) (((indexW - indexR) & INDEX_MASK) / 2);
	m_numLeftI = (numLeft + m_numLeftI*(CONTROL_AVG - 1)) / CONTROL_AVG;
	float offset = (m_numLeftI - LOW_WATERMARK) * CONTROL_FACTOR;
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

	float ratio = aid_sample_rate / (float) m_mixer->m_sampleRate;

	float lvolume = (float) m_LVolume / 256.f;
	float rvolume = (float) m_RVolume / 256.f;
	s32 lIvolume = m_LVolume;
	s32 rIvolume = m_RVolume;

	m_errorL1 = 0, m_errorL2 = 0;
	m_errorR1 = 0, m_errorR2 = 0;
	m_randR1 = 0, m_randR2 = 0, m_randR1 = 0, m_randR2 = 0;
	float templ, tempr;

	for (; currentSample < numSamples * 2 && ((indexW - indexR) & INDEX_MASK) > 2; currentSample += 2) {
		// get sinc table with *closest* desired offset
		int index = (int) (m_frac * SINC_FSIZE);
		const float* table0 = m_sinc_table[index];

		// again, don't need sample -2 because it'll always be multiplied by 0
		u32 indexRp = indexR - 2; // sample -1
		// indexR is sample 0
		u32 indexR2 = indexR + 2; // sample  1
		u32 indexR4 = indexR + 4; // sample  2

		// LEFT CHANNEL
		float sl1 = float_buffer[(indexRp) & INDEX_MASK];
		float sl2 = float_buffer[(indexR) & INDEX_MASK];
		float sl3 = float_buffer[(indexR2) & INDEX_MASK];
		float sl4 = float_buffer[(indexR4) & INDEX_MASK];
		
		float al = sl1 * table0[0];
		float bl = sl2 * table0[1];
		float cl = sl3 * table0[2];
		float dl = sl4 * table0[3];
		float sampleL = al + bl + cl + dl;
		
		//float sampleL = l[0];
		sampleL = sampleL * lvolume;
		sampleL += twos2float(samples[currentSample + 1]);

		// dither
		m_randL2 = m_randL1;
		m_randL1 = rand();
		templ = sampleL + DITHER_SHAPE * (m_errorL1 + m_errorL1 - m_errorL2);
		sampleL = templ + DITHER_OFFSET + DITHER_SIZE * (float) (m_randL1 - m_randL2);

		// clamp and output
		MathUtil::Clamp(&sampleL, -1.f, 1.f);
		int sampleLi = float2stwos(sampleL);
		samples[currentSample + 1] = sampleLi;

		// update dither accumulators
		m_errorL2 = m_errorL1;
		m_errorL1 = templ - sampleL;

		// RIGHT CHANNEL
		float sr1 = float_buffer[(indexRp + 1) & INDEX_MASK];
		float sr2 = float_buffer[(indexR + 1) & INDEX_MASK];
		float sr3 = float_buffer[(indexR2 + 1) & INDEX_MASK];
		float sr4 = float_buffer[(indexR4 + 1) & INDEX_MASK];
		
		float ar = sr1 * table0[0];
		float br = sr2 * table0[1];
		float cr = sr3 * table0[2];
		float dr = sr4 * table0[3];
		float sampleR = ar + br + cr + dr;
		

		//float sampleR = r[0];
		sampleR = sampleR * rvolume;
		sampleR += twos2float(samples[currentSample]);

		m_randR2 = m_randR1;
		m_randR1 = rand();
		tempr = sampleR + DITHER_SHAPE * (m_errorR1 + m_errorR1 - m_errorR2);
		sampleR = tempr + DITHER_OFFSET + DITHER_SIZE * (float) (m_randR1 - m_randR2);

		MathUtil::Clamp(&sampleR, -1.f, 1.f);
		int sampleRi = float2stwos(sampleR);
		samples[currentSample] = sampleRi;

		m_errorR2 = m_errorR1;
		m_errorR1 = tempr - sampleR;

		m_frac += ratio;
		indexR += 2 * (int) m_frac;
		m_frac = m_frac - (int) m_frac;
	}

	// Padding
	short s[2];
	s[0] = Common::swap16(m_buffer[(indexR - 1) & INDEX_MASK]);
	s[1] = Common::swap16(m_buffer[(indexR - 2) & INDEX_MASK]);
	s[0] = (s[0] * rIvolume) >> 8;
	s[1] = (s[1] * lIvolume) >> 8;
	for (; currentSample < numSamples * 2; currentSample += 2)
	{
		int sampleR = s[0] + samples[currentSample];
		MathUtil::Clamp(&sampleR, -32768, 32767);
		samples[currentSample] = sampleR;
		int sampleL = s[1] + samples[currentSample + 1];
		MathUtil::Clamp(&sampleL, -32768, 32767);
		samples[currentSample + 1] = sampleL;
	}

	// Flush cached variable
	Common::AtomicStore(m_indexR, indexR);

	return numSamples;
}

unsigned int CMixer::Mix(short* samples, unsigned int num_samples, bool consider_framelimit)
{
	if (!samples)
		return 0;

	std::lock_guard<std::mutex> lk(m_csMixing);

	memset(samples, 0, num_samples * 2 * sizeof(short));

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

void CMixer::MixerFifo::PushSamples(const short *samples, unsigned int num_samples)
{
	// Cache access in non-volatile variable
	// indexR isn't allowed to cache in the audio throttling loop as it
	// needs to get updates to not deadlock.
	u32 indexW = Common::AtomicLoad(m_indexW);
	m_previousW = indexW;

	// Check if we have enough free space
	// indexW == m_indexR results in empty buffer, so indexR must always be smaller than indexW
	if (num_samples * 2 + ((indexW - Common::AtomicLoad(m_indexR)) & INDEX_MASK) >= MAX_SAMPLES * 2)
		return;

	// AyuanX: Actual re-sampling work has been moved to sound thread
	// to alleviate the workload on main thread
	// and we simply store raw data here to make fast mem copy
	int over_bytes = num_samples * 4 - (MAX_SAMPLES * 2 - (indexW & INDEX_MASK)) * sizeof(short);
	if (over_bytes > 0)
	{
		memcpy(&m_buffer[indexW & INDEX_MASK], samples, num_samples * 4 - over_bytes);
		memcpy(&m_buffer[0], samples + (num_samples * 4 - over_bytes) / sizeof(short), over_bytes);
	}
	else
	{
		memcpy(&m_buffer[indexW & INDEX_MASK], samples, num_samples * 4);
	}

	Common::AtomicAdd(m_indexW, num_samples * 2);

	indexW = Common::AtomicLoad(m_indexW);
	populateFloats(m_previousW, indexW);

	return;
}

void CMixer::PushSamples(const short *samples, unsigned int num_samples)
{
	m_dma_mixer.PushSamples(samples, num_samples);
	if (m_log_dsp_audio)
		g_wave_writer_dsp.AddStereoSamplesBE(samples, num_samples);
}

void CMixer::PushStreamingSamples(const short *samples, unsigned int num_samples)
{
	m_streaming_mixer.PushSamples(samples, num_samples);
	if (m_log_dtk_audio)
		g_wave_writer_dtk.AddStereoSamplesBE(samples, num_samples);
}

void CMixer::PushWiimoteSpeakerSamples(const short *samples, unsigned int num_samples, unsigned int sample_rate)
{
	short samples_stereo[MAX_SAMPLES * 2];

	if (num_samples < MAX_SAMPLES)
	{
		m_wiimote_speaker_mixer.SetInputSampleRate(sample_rate);

		for (unsigned int i = 0; i < num_samples; ++i)
		{
			samples_stereo[i * 2] = Common::swap16(samples[i]);
			samples_stereo[i * 2 + 1] = Common::swap16(samples[i]);
		}

		m_wiimote_speaker_mixer.PushSamples(samples_stereo, num_samples);
	}
}

void CMixer::SetDMAInputSampleRate(unsigned int rate)
{
	m_dma_mixer.SetInputSampleRate(rate);
}

void CMixer::SetStreamInputSampleRate(unsigned int rate)
{
	m_streaming_mixer.SetInputSampleRate(rate);
}

void CMixer::SetStreamingVolume(unsigned int lvolume, unsigned int rvolume)
{
	m_streaming_mixer.SetVolume(lvolume, rvolume);
}

void CMixer::SetWiimoteSpeakerVolume(unsigned int lvolume, unsigned int rvolume)
{
	m_wiimote_speaker_mixer.SetVolume(lvolume, rvolume);
}

void CMixer::MixerFifo::SetInputSampleRate(unsigned int rate)
{
	m_input_sample_rate = rate;
}

void CMixer::MixerFifo::SetVolume(unsigned int lvolume, unsigned int rvolume)
{
	m_LVolume = lvolume + (lvolume >> 7);
	m_RVolume = rvolume + (rvolume >> 7);
}

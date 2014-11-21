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

#define _USE_MATH_DEFINES
#include <math.h>

#if _M_SSE >= 0x301 && !(defined __GNUC__ && !defined __SSSE3__)
#include <tmmintrin.h>
#endif

// converts [-32768, 32767] -> [-1, 1]
inline float Signed16ToFloat(s16 s)
{
	return (s > 0) ? (float) (s / (float) 0x7fff) : (float) (s / (float) 0x8000);
}

// converts [-1, 1] -> [-32768, 32767]
inline s16 FloatToSigned16(float f)
{
	return (f > 0) ? (s16) (f * 0x7fff) : (s16) (f * 0x8000);
}

inline float SincSinc(float x, float window_width)
{
	float pi_x = (float) M_PI * x;
	return (float) (window_width * sin(pi_x) * sin(pi_x / window_width) / (pi_x * pi_x));
}

//void CMixer::MixerFifo::PopulateFloats(u32 start, u32 stop)
//{
//	for (u32 i = start; i < stop; ++i)
//	{
//		m_float_buffer[i & INDEX_MASK] = Signed16ToFloat(Common::swap16(m_buffer[i & INDEX_MASK]));
//	}
//}

void CMixer::PopulateSincTable() {
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

void CMixer::MixerFifo::MixLinear(std::vector<float>& samples, u32 numSamples, bool consider_framelimit)
{
	u32 current_sample = 0;

	// Cache access in non-volatile variable
	// Without this cache, the compiler wouldn't be allowed to optimize the
	// interpolation loop.
	u32 r_index = Common::AtomicLoad(m_r_index);
	u32 w_index = Common::AtomicLoad(m_w_index);

	float num_left = (float) (((w_index - r_index) & INDEX_MASK) / 2);
	m_num_left_i = (num_left + m_num_left_i*(CONTROL_AVG - 1)) / CONTROL_AVG;
	float offset = (m_num_left_i - LOW_WATERMARK) * CONTROL_FACTOR;
	if (offset > MAX_FREQ_SHIFT) offset = MAX_FREQ_SHIFT;
	if (offset < -MAX_FREQ_SHIFT) offset = -MAX_FREQ_SHIFT;

	// render numleft sample pairs to samples[]
	// advance indexR with sample position
	// remember fractional offset

	u32 framelimit = SConfig::GetInstance().m_Framelimit;
	float aid_sample_rate = m_input_sample_rate + offset;
	if (consider_framelimit && framelimit > 1)
	{
		aid_sample_rate = aid_sample_rate * (framelimit - 1) * 5 / VideoInterface::TargetRefreshRate;
	}

	float ratio = aid_sample_rate / (float) m_mixer->m_sample_rate;

	float l_volume = (float) m_lvolume / 255.f;
	float r_volume = (float) m_rvolume / 255.f;

	for (; current_sample < numSamples * 2 && ((w_index - r_index) & INDEX_MASK) > 2; current_sample += 2)
	{
		samples[current_sample + 1] += l_volume * (m_float_buffer[r_index & INDEX_MASK] + (m_float_buffer[(r_index + 2) & INDEX_MASK] - m_float_buffer[r_index & INDEX_MASK]) * m_fraction);
		samples[current_sample] += r_volume * (m_float_buffer[(r_index + 1) & INDEX_MASK] + (m_float_buffer[(r_index + 3) & INDEX_MASK] - m_float_buffer[(r_index + 1) & INDEX_MASK]) * m_fraction);
	
		m_fraction += ratio;
		r_index += 2 * (int) m_fraction;
		m_fraction = m_fraction - (int) m_fraction;
	}

	float s[2];
	s[0] = m_float_buffer[(r_index - 1) & INDEX_MASK] * r_volume;
	s[1] = m_float_buffer[(r_index - 2) & INDEX_MASK] * l_volume;
	for (; current_sample < numSamples * 2; current_sample += 2)
	{
		samples[current_sample] += s[0];
		samples[current_sample + 1] += s[1];
	}
	Common::AtomicStore(m_r_index, r_index);

}
// Executed from sound stream thread
void CMixer::MixerFifo::Mix(std::vector<float>& samples, u32 numSamples, bool consider_framelimit)
{
	u32 current_sample = 0;

	// Cache access in non-volatile variable
	// Without this cache, the compiler wouldn't be allowed to optimize the
	// interpolation loop.
	u32 r_index = Common::AtomicLoad(m_r_index);
	u32 w_index = Common::AtomicLoad(m_w_index);

	float num_left = (float) (((w_index - r_index) & INDEX_MASK) / 2);
	m_num_left_i = (num_left + m_num_left_i*(CONTROL_AVG - 1)) / CONTROL_AVG;
	float offset = (m_num_left_i - LOW_WATERMARK) * CONTROL_FACTOR;
	if (offset > MAX_FREQ_SHIFT) offset = MAX_FREQ_SHIFT;
	if (offset < -MAX_FREQ_SHIFT) offset = -MAX_FREQ_SHIFT;

	// render numleft sample pairs to samples[]
	// advance indexR with sample position
	// remember fractional offset

	u32 framelimit = SConfig::GetInstance().m_Framelimit;
	float aid_sample_rate = m_input_sample_rate + offset;
	if (consider_framelimit && framelimit > 1)
	{
		aid_sample_rate = aid_sample_rate * (framelimit - 1) * 5 / VideoInterface::TargetRefreshRate;
	}

	float ratio = aid_sample_rate / (float) m_mixer->m_sample_rate;

	float l_volume = (float) m_lvolume / 255.f;
	float r_volume = (float) m_rvolume / 255.f;
	
	for (; current_sample < numSamples * 2 && ((w_index - r_index) & INDEX_MASK) > 2; current_sample += 2)
	{
		// get sinc table with floor(closest) desired offset
		s32 index = (s32) (m_fraction * SINC_FSIZE);
		const std::vector<float>& weights = m_mixer->m_sinc_table[index];

		// interpolate
		std::vector<float> l_samples(SINC_SIZE);
		std::vector<float> r_samples(SINC_SIZE);
		for (u32 i = 0; i < SINC_SIZE; ++i)
		{
			u32 current_index = r_index + (2 * i) - (SINC_SIZE / 2);
			l_samples[i] = m_float_buffer[current_index       & INDEX_MASK] * weights[i];
			r_samples[i] = m_float_buffer[(current_index + 1) & INDEX_MASK] * weights[i];
		}

		float l_output = 0;
		float r_output = 0;
		for (u32 i = 0; i < SINC_SIZE; ++i)
		{
			l_output += l_samples[i];
			r_output += r_samples[i];
		}
		
		l_output = l_output * l_volume;
		samples[current_sample + 1] += l_output;

		r_output = r_output * r_volume;
		samples[current_sample] += r_output;

		m_fraction += ratio;
		r_index += 2 * (int) m_fraction;
		m_fraction = m_fraction - (int) m_fraction;
	}
	
	float s[2];
	s[0] = m_float_buffer[(r_index - 1) & INDEX_MASK] * r_volume;
	s[1] = m_float_buffer[(r_index - 2) & INDEX_MASK] * l_volume;
	for (; current_sample < numSamples * 2; current_sample += 2)
	{
		samples[current_sample    ] += s[0];
		samples[current_sample + 1] += s[1];
	}

	// Flush cached variable
	Common::AtomicStore(m_r_index, r_index);

}

u32 CMixer::Mix(s16* samples, u32 num_samples, bool consider_framelimit)
{
	if (!samples)
		return 0;

	std::lock_guard<std::mutex> lk(m_cs_mixing);

	if (PowerPC::GetState() != PowerPC::CPU_RUNNING)
	{
		// Silence
		memset(samples, 0, num_samples * 2 * sizeof(s16));
		return num_samples;
	}

	m_output_buffer.resize(num_samples * 2);
	std::fill_n(m_output_buffer.begin(), num_samples * 2, 0.f);
	
	//m_dma_mixer.Mix(m_output_buffer, num_samples, consider_framelimit);
	//m_streaming_mixer.Mix(m_output_buffer, num_samples, consider_framelimit);

	// adding wiimote speaker runs risk of distortion (getting |values| > 1)
	// we trust |dma + streaming| < 1.
	// use logarithmic compression for now
	u32 wiimote_lvolume = 0, wiimote_rvolume = 0;
	m_wiimote_speaker_mixer.GetVolume(&wiimote_lvolume, &wiimote_rvolume);
	if (wiimote_lvolume && wiimote_rvolume)
	{
		float highest_volume = std::max((float) wiimote_lvolume / 255, (float) wiimote_rvolume / 255);
		m_wiimote_speaker_mixer.Mix(m_output_buffer, num_samples, consider_framelimit);
		for (float sample : m_output_buffer)
		{
			if (abs(sample) > COMPRESS_THRESH)
			{
				float compress = (float) log(1 + COMPRESS_ALPHA * (abs(sample) - COMPRESS_THRESH) / (1 + highest_volume - COMPRESS_THRESH));
				compress /= (float) log(1 + COMPRESS_ALPHA);
				sample = sample / abs(sample) * (COMPRESS_THRESH + (1 - COMPRESS_THRESH) * compress);
			}
		}
	}

	for (u32 i = 0; i < num_samples * 2; i += 2) {
		// dither
		float l_output = m_output_buffer[i + 1];
		float r_output = m_output_buffer[i];

		l_rand2 = l_rand1;
		l_rand1 = rand();
		float l_shape = l_output + DITHER_SHAPE * (l_error1 + l_error1 - l_error2);
		l_output = l_shape + DITHER_OFFSET + DITHER_SIZE * (float) (l_rand1 - l_rand2);

		r_rand2 = r_rand1;
		r_rand1 = rand();
		float r_shape = r_output + DITHER_SHAPE * (r_error1 + r_error1 - r_error2);
		r_output = r_shape + DITHER_OFFSET + DITHER_SIZE * (float) (r_rand1 - r_rand2);

		// update dither accumulators
		l_error2 = l_error1;
		l_error1 = l_shape - l_output;

		r_error2 = r_error1;
		r_error1 = r_shape - r_output;

		// clamp and output
		MathUtil::Clamp(&l_output, -1.f, 1.f);
		samples[i + 1] = FloatToSigned16(l_output);
		
		MathUtil::Clamp(&r_output, -1.f, 1.f);
		samples[i] = FloatToSigned16(r_output);
		INFO_LOG(DSPHLE, "%d ", samples[i]);
		INFO_LOG(DSPHLE, "%d ", samples[i + 1]);
	}

	return num_samples;
}

void CMixer::MixerFifo::PushSamples(const s16 *samples, u32 num_samples)
{ 
	// Cache access in non-volatile variable
	// indexR isn't allowed to cache in the audio throttling loop as it
	// needs to get updates to not deadlock.
	u32 current_w_index = Common::AtomicLoad(m_w_index);
	u32 previous_w_index = current_w_index;

	// Check if we have enough free space
	// indexW == m_indexR results in empty buffer, so indexR must always be smaller than indexW
	if (num_samples * 2 + ((current_w_index - Common::AtomicLoad(m_r_index)) & INDEX_MASK) >= MAX_SAMPLES * 2)
		return;

	// AyuanX: Actual re-sampling work has been moved to sound thread
	// to alleviate the workload on main thread
	// and we simply store raw data here to make fast mem copy
	for (u32 i = 0; i < num_samples * 2; ++i)
	{
		m_float_buffer[(current_w_index + i) & INDEX_MASK] = Signed16ToFloat(Common::swap16(samples[i]));
	}

	Common::AtomicAdd(m_w_index, num_samples * 2);

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
			//samples_stereo[i * 2 + 1] = Common::swap16(samples[i]);
			samples_stereo[i * 2 + 1] = 0;
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
	m_lvolume = lvolume;
	m_rvolume = rvolume;
}

void CMixer::MixerFifo::GetVolume(u32* lvolume, u32* rvolume) const
{
	*lvolume = m_lvolume;
	*rvolume = m_rvolume;
}
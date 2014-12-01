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

// converts [-32768, 32767] -> [-1.0, 1.0]
inline float CMixer::Signed16ToFloat(s16 s)
{
	return (s > 0) ? (float) (s / (float) 0x7fff) : (float) (s / (float) 0x8000);
}

// converts [-1.0, 1.0] -> [-32768, 32767]
inline s16 CMixer::FloatToSigned16(float f)
{
	return (f > 0) ? (s16) (f * 0x7fff) : (s16) (f * 0x8000);
}

inline float CMixer::LinearInterpolate(const float s0, const float s1, const float t)
{
	return (1 - t) * s0 + t * s1;
}

// Linear interpolation seems to be the best for Wiimote 3khz -> 48khz, for now.
// TODO: figure out why and make it work with the above FIR
void CMixer::MixerFifo::MixLinear(std::vector<float>& samples, u32 numSamples, bool consider_framelimit)
{
	u32 current_sample = 0;

	// Cache access in non-volatile variable so interpolation loop can be optimized
	u32 read_index = Common::AtomicLoad(m_read_index);
	const u32 write_index = Common::AtomicLoad(m_write_index);

	// Sync input rate by fifo size
	float num_left = (float) (((write_index - read_index) & INDEX_MASK) / 2);
	m_num_left_i   = (num_left + m_num_left_i * (CONTROL_AVG - 1)) / CONTROL_AVG;
	float offset   = (m_num_left_i - LOW_WATERMARK) * CONTROL_FACTOR;
	if (offset > MAX_FREQ_SHIFT)
		offset = MAX_FREQ_SHIFT;
	if (offset < -MAX_FREQ_SHIFT)
		offset = -MAX_FREQ_SHIFT;

	// adjust framerate with framelimit
	u32 framelimit = SConfig::GetInstance().m_Framelimit;
	float aid_sample_rate = m_input_sample_rate + offset;
	if (consider_framelimit && framelimit > 1)
	{
		aid_sample_rate = aid_sample_rate * (framelimit - 1) * 5 / VideoInterface::TargetRefreshRate;
	}

	// ratio = 1 / upscale_factor = stepsize for each sample
	// e.g. going from 32khz to 48khz is 1 / (3 / 2) = 2 / 3
	// note because of syncing and framelimit, ratio will rarely be exactly 2 / 3
	float ratio = aid_sample_rate / (float) m_mixer->m_sample_rate;

	float l_volume = (float) m_lvolume / 255.f;
	float r_volume = (float) m_rvolume / 255.f;

	// for each output sample pair (left and right),
	//   linear interpolate between current and next sample
	//   increment output sample position
	//   increment input sample position by ratio, store fraction
	for (; current_sample < numSamples * 2 && ((write_index - read_index) & INDEX_MASK) > 0; current_sample += 2)
	{
		float l_output = LinearInterpolate(m_float_buffer[read_index       & INDEX_MASK],
		                                   m_float_buffer[(read_index + 2) & INDEX_MASK],
										   m_fraction);
		float r_output = LinearInterpolate(m_float_buffer[(read_index + 1) & INDEX_MASK],
			                               m_float_buffer[(read_index + 3) & INDEX_MASK],
										   m_fraction);

		samples[current_sample + 1] += l_volume * l_output;
		samples[current_sample    ] += r_volume * r_output;
	
		m_fraction += ratio;
		read_index += 2 * (int) m_fraction;
		m_fraction  = m_fraction - (int) m_fraction;
	}

	// pad output if not enough input samples
	float s[2];
	s[0] = m_float_buffer[(read_index - 1) & INDEX_MASK] * r_volume;
	s[1] = m_float_buffer[(read_index - 2) & INDEX_MASK] * l_volume;
	for (; current_sample < numSamples * 2; current_sample += 2)
	{
		samples[current_sample    ] += s[0];
		samples[current_sample + 1] += s[1];
	}

	// update read index
	Common::AtomicStore(m_read_index, read_index);
}
// Executed from sound stream thread
void CMixer::MixerFifo::Mix(std::vector<float>& samples, u32 numSamples, bool consider_framelimit)
{
	u32 current_sample = 0;

	// Cache access in non-volatile variable so interpolation loop can be optimized
	u32 read_index = Common::AtomicLoad(m_read_index);
	const u32 write_index = Common::AtomicLoad(m_write_index);

	// Sync input rate by fifo size
	float num_left = (float) (((write_index - read_index) & INDEX_MASK) / 2);
	m_num_left_i   = (num_left + m_num_left_i * (CONTROL_AVG - 1)) / CONTROL_AVG;
	float offset   = (m_num_left_i - LOW_WATERMARK) * CONTROL_FACTOR;
	if (offset > MAX_FREQ_SHIFT)
		offset = MAX_FREQ_SHIFT;
	if (offset < -MAX_FREQ_SHIFT)
		offset = -MAX_FREQ_SHIFT;

	// adjust framerate with framelimit
	u32 framelimit = SConfig::GetInstance().m_Framelimit;
	float aid_sample_rate = m_input_sample_rate + offset;
	if (consider_framelimit && framelimit > 1)
	{
		aid_sample_rate = aid_sample_rate * (framelimit - 1) * 5 / VideoInterface::TargetRefreshRate;
	}

	float ratio = aid_sample_rate / (float) m_mixer->m_sample_rate;

	float l_volume = (float) m_lvolume / 255.f;
	float r_volume = (float) m_rvolume / 255.f;

	// for each output sample pair (left and right),
	//   since filter table is one-sided,
	//     convolve input samples to the left with filter
	//     then convolve input samples to the right
	//   increment output sample position
	//   increment input sample position by ratio, store fraction
	//
	// see https://ccrma.stanford.edu/~jos/resample/Implementation.html
	//
	for (; current_sample < numSamples * 2 && ((write_index - read_index) & INDEX_MASK) > 0; current_sample += 2)
	{
		double left_output = 0, right_output = 0;

		// left wing of filter
		double left_wing_fraction = (m_fraction * SAMPLES_PER_CROSSING);
		u32 left_wing_index = (u32) left_wing_fraction;
		left_wing_fraction -= left_wing_index;

		const Resampler& resampler = m_mixer->m_resampler;
		u32 current_index = read_index;
		while (left_wing_index < resampler.m_lowpass_filter.size())
		{
			double impulse = resampler.m_lowpass_filter[left_wing_index];
			impulse += resampler.m_lowpass_delta[left_wing_index] * left_wing_fraction;

			left_output  += (float) m_float_buffer[ current_index      & INDEX_MASK] * impulse;
			right_output += (float) m_float_buffer[(current_index + 1) & INDEX_MASK] * impulse;

			left_wing_index += SAMPLES_PER_CROSSING;
			current_index -= 2;
		}

		// right wing of filter
		double right_wing_fraction = (1 - m_fraction) * SAMPLES_PER_CROSSING;
		u32 right_wing_index = ((u32) right_wing_fraction) % SAMPLES_PER_CROSSING;
		right_wing_fraction -= right_wing_index;

		// we already used read_index for left wing
		current_index = read_index + 2;
		while (right_wing_index < resampler.m_lowpass_filter.size())
		{
			double impulse = resampler.m_lowpass_filter[right_wing_index];
			impulse += resampler.m_lowpass_delta[right_wing_index] * right_wing_fraction;

			left_output  += (float) m_float_buffer[ current_index      & INDEX_MASK] * impulse;
			right_output += (float) m_float_buffer[(current_index + 1) & INDEX_MASK] * impulse;

			right_wing_index += SAMPLES_PER_CROSSING;
			current_index += 2;
		}
		
		left_output = left_output * l_volume;
		samples[current_sample + 1] += (float)left_output;

		right_output = right_output * r_volume;
		samples[current_sample] += (float) right_output;

		m_fraction += ratio;
		read_index += 2 * (s32) m_fraction;
		m_fraction  = m_fraction - (s32) m_fraction;
	}
	
	float s[2];
	s[0] = m_float_buffer[(read_index - 1) & INDEX_MASK] * r_volume;
	s[1] = m_float_buffer[(read_index - 2) & INDEX_MASK] * l_volume;
	for (; current_sample < numSamples * 2; current_sample += 2)
	{
		samples[current_sample    ] += s[0];
		samples[current_sample + 1] += s[1];
	}

	// Flush cached variable
	Common::AtomicStore(m_read_index, read_index);
}

// we NEED dithering going from float -> 16bit
void CMixer::TriangleDither(float* l_sample, float* r_sample)
{
	float left_dither = DITHER_NOISE;
	float right_dither = DITHER_NOISE;
	*l_sample = (*l_sample) + left_dither - m_l_dither_prev;
	*r_sample = (*r_sample) + right_dither - m_r_dither_prev;
	m_l_dither_prev = left_dither;
	m_r_dither_prev = right_dither;
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

	// reset float output buffer
	m_output_buffer.resize(num_samples * 2);
	std::fill_n(m_output_buffer.begin(), num_samples * 2, 0.f);
	
	m_dma_mixer.Mix(m_output_buffer, num_samples, consider_framelimit);
	m_streaming_mixer.Mix(m_output_buffer, num_samples, consider_framelimit);
	m_wiimote_speaker_mixer.MixLinear(m_output_buffer, num_samples, consider_framelimit);

	// dither and clamp
	for (u32 i = 0; i < num_samples * 2; i += 2) {
	
		float l_output = m_output_buffer[i + 1];
		float r_output = m_output_buffer[i];
		TriangleDither(&m_output_buffer[i + 1], &m_output_buffer[i]);

		MathUtil::Clamp(&l_output, -1.f, 1.f);
		samples[i + 1] = FloatToSigned16(l_output);
		
		MathUtil::Clamp(&r_output, -1.f, 1.f);
		samples[i] = FloatToSigned16(r_output);
	}

	return num_samples;
}

void CMixer::MixerFifo::PushSamples(const s16 *samples, u32 num_samples)
{ 
	// Cache access in non-volatile variable
	// indexR isn't allowed to cache in the audio throttling loop as it
	// needs to get updates to not deadlock.
	u32 current_write_index = Common::AtomicLoad(m_write_index);

	// Check if we have enough free space
	// indexW == m_indexR results in empty buffer, so indexR must always be smaller than indexW
	if (num_samples * 2 + ((current_write_index - Common::AtomicLoad(m_read_index)) & INDEX_MASK) >= MAX_SAMPLES * 2)
		return;

	// AyuanX: Actual re-sampling work has been moved to sound thread
	// to alleviate the workload on main thread
	// convert to float while copying to buffer
	for (u32 i = 0; i < num_samples * 2; ++i)
	{
		m_float_buffer[(current_write_index + i) & INDEX_MASK] = Signed16ToFloat(Common::swap16(samples[i]));
	}

	Common::AtomicAdd(m_write_index, num_samples * 2);

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
	m_lvolume = lvolume;
	m_rvolume = rvolume;
}

void CMixer::MixerFifo::GetVolume(u32* lvolume, u32* rvolume) const
{
	*lvolume = m_lvolume;
	*rvolume = m_rvolume;
}

// I_0(x) = summation((((x/2)^k) / k!)^2) for k from 0 to Infinity
double CMixer::Resampler::ModBessel0th(const double x)
{
	double sum = 1;
	int factorial_store = 1;
	double half_x = x / 2.f;
	double previous = 1;
	do {
		double temp = half_x / (double) factorial_store;
		temp *= temp;
		previous *= temp;
		sum += previous;
		factorial_store++;
	} while (previous >= BESSEL_EPSILON * sum);
	return sum;
}

// one wing of FIR by using sinc * Kaiser window
void CMixer::Resampler::PopulateFilterCoeff()
{
	// Generate sinc table
	m_lowpass_filter[0] = LOWPASS_ROLLOFF;
	for (s32 i = 1; i < m_lowpass_filter.size(); ++i)
	{
		double temp = M_PI * (double) i / SAMPLES_PER_CROSSING;
		m_lowpass_filter[i] = sin(temp * LOWPASS_ROLLOFF) / temp;
	}

	// use a Kaiser window
	// https://ccrma.stanford.edu/~jos/sasp/Kaiser_Window.html
	//
	double I0_beta = 1.0 / ModBessel0th(KAISER_BETA);
	double inside = 1.0 / (m_lowpass_filter.size() - 1);
	for (s32 i = 1; i < m_lowpass_filter.size(); ++i)
	{
		double temp = (double) i * inside;
		temp = 1.0 - temp * temp;
		temp = (temp < 0) ? 0 : temp;
		m_lowpass_filter[i] *= ModBessel0th(KAISER_BETA * sqrt(temp)) * I0_beta;
	}

	// store deltas in delta table for faster lookup to interpolate impulse
	for (u32 i = 0; i < m_lowpass_filter.size() - 1; ++i)
	{
		m_lowpass_delta[i] = m_lowpass_filter[i + 1] - m_lowpass_filter[i];
	}
	m_lowpass_delta.back() = -1 * m_lowpass_filter.back();

}

// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2
// Refer to the license.txt file included.

#pragma once

#include <mutex>
#include <string>

#include "AudioCommon/WaveFile.h"

// 16 bit Stereo
#define MAX_SAMPLES     (1024 * 2) // 64ms
#define INDEX_MASK      (MAX_SAMPLES * 2 - 1)

#define LOW_WATERMARK   1280 // 40 ms
#define MAX_FREQ_SHIFT  200  // per 32000 Hz
#define CONTROL_FACTOR  0.2f // in freq_shift per fifo size offset
#define CONTROL_AVG     32

// Lanczos table
#define SINC_FSIZE		65536	// sinc table granularity = 1 / SINC_FSIZE.
#define SINC_SIZE		(5 - 1) // see comment for populate_sinc_table()

// Dither defines
#define DITHER_SHAPE	0.5f
#define DITHER_WORD		(0xFFFF)
#define DITHER_WIDTH	(1.f / DITHER_WORD)
#define DITHER_SIZE		(DITHER_WIDTH / RAND_MAX)
#define DITHER_OFFSET	(DITHER_WIDTH * DITHER_SHAPE)

#define DITHER_BUFFER_SIZE 8
#define DITHER_NOISE       (rand() / (float) RAND_MAX - 0.5f)
#define DITHER_BUFFER_MASK 7

// Bessel defines
#define BESSEL_EPSILON 1e-21

class CMixer {

public:
	CMixer(u32 BackendSampleRate)
		: m_dma_mixer(this, 32000)
		, m_streaming_mixer(this, 48000)
		, m_wiimote_speaker_mixer(this, 3000)
		, m_sample_rate(BackendSampleRate)
		, m_resampler()
		, m_log_dtk_audio(0)
		, m_log_dsp_audio(0)
		, m_speed(0)
		, l_rand1(0), l_rand2(0)
		, r_rand1(0), r_rand2(0)
		, l_error1(0), l_error2(0)
		, r_error1(0), r_error2(0)
		, m_phase(0)
	{
		INFO_LOG(AUDIO_INTERFACE, "Mixer is initialized");
		m_output_buffer.reserve(MAX_SAMPLES * 2);
		m_dither_buffer.resize(DITHER_BUFFER_SIZE * 2, 0.f);
		m_sinc_table.resize(SINC_FSIZE, std::vector<float>(SINC_SIZE, 0));
		PopulateSincTable();
		const float SHAPED_BS[5] = { 2.033f, -2.165f, 1.959f, -1.590f, 0.6149f };
		size_t size = m_resampler.m_lowpass_filter.size();
	}

	virtual ~CMixer() {}

	// Called from audio threads
	u32 Mix(s16* samples, u32 numSamples, bool consider_framelimit = true);

	// Called from main thread
	virtual void PushSamples(const s16* samples, u32 num_samples);
	virtual void PushStreamingSamples(const s16* samples, u32 num_samples);
	virtual void PushWiimoteSpeakerSamples(const s16* samples, u32 num_samples, u32 sample_rate);
	u32 GetSampleRate() const { return m_sample_rate; }

	void SetDMAInputSampleRate(u32 rate);
	void SetStreamInputSampleRate(u32 rate);
	void SetStreamingVolume(u32 lvolume, u32 rvolume);
	void SetWiimoteSpeakerVolume(u32 lvolume, u32 rvolume);

	virtual void StartLogDTKAudio(const std::string& filename)
	{
		if (!m_log_dtk_audio)
		{
			m_log_dtk_audio = true;
			g_wave_writer_dtk.Start(filename, 48000);
			g_wave_writer_dtk.SetSkipSilence(false);
			NOTICE_LOG(DSPHLE, "Starting DTK Audio logging");
		}
		else
		{
			WARN_LOG(DSPHLE, "DTK Audio logging has already been started");
		}
	}

	virtual void StopLogDTKAudio()
	{
		if (m_log_dtk_audio)
		{
			m_log_dtk_audio = false;
			g_wave_writer_dtk.Stop();
			NOTICE_LOG(DSPHLE, "Stopping DTK Audio logging");
		}
		else
		{
			WARN_LOG(DSPHLE, "DTK Audio logging has already been stopped");
		}
	}

	virtual void StartLogDSPAudio(const std::string& filename)
	{
		if (!m_log_dsp_audio)
		{
			m_log_dsp_audio = true;
			g_wave_writer_dsp.Start(filename, 32000);
			g_wave_writer_dsp.SetSkipSilence(false);
			NOTICE_LOG(DSPHLE, "Starting DSP Audio logging");
		}
		else
		{
			WARN_LOG(DSPHLE, "DSP Audio logging has already been started");
		}
	}

	virtual void StopLogDSPAudio()
	{
		if (m_log_dsp_audio)
		{
			m_log_dsp_audio = false;
			g_wave_writer_dsp.Stop();
			NOTICE_LOG(DSPHLE, "Stopping DSP Audio logging");
		}
		else
		{
			WARN_LOG(DSPHLE, "DSP Audio logging has already been stopped");
		}
	}

	std::mutex& MixerCritical() { return m_cs_mixing; }

	float GetCurrentSpeed() const { return m_speed; }
	void UpdateSpeed(volatile float val) { m_speed = val; }
	void PopulateSincTable();

protected:
	class MixerFifo {
	public:
		MixerFifo(CMixer* mixer, u32 sample_rate)
			: m_mixer(mixer)
			, m_input_sample_rate(sample_rate)
			, m_w_index(0)
			, m_r_index(0)
			, m_lvolume(255)
			, m_rvolume(255)
			, m_num_left_i(0.0f)
			, m_fraction(0.0f)
		{
			srand((u32) time(NULL));
			//m_buffer.resize(MAX_SAMPLES * 2, 0);
			m_float_buffer.resize(MAX_SAMPLES * 2, 0);
		}
		void PushSamples(const s16* samples, u32 num_samples);
		void Mix(std::vector<float>& samples, u32 numSamples, bool consider_framelimit = true);
		void MixLinear(std::vector<float>& samples, u32 numSamples, bool consider_framelimit);
		void SetInputSampleRate(u32 rate);
		void SetVolume(u32 lvolume, u32 rvolume);
		void GetVolume(u32* lvolume, u32* rvolume) const;
		//void PopulateFloats(u32 start, u32 stop);

	private:
		CMixer*  m_mixer;
		u32      m_input_sample_rate;
		//std::vector<s16>                 m_buffer;       // [MAX_SAMPLES * 2];
		std::vector<float>               m_float_buffer; // [MAX_SAMPLES * 2];

		volatile u32 m_w_index;
		volatile u32 m_r_index;
		float    m_num_left_i;
		float    m_fraction;

		// Volume ranges from 0-255
		volatile u32 m_rvolume;
		volatile u32 m_lvolume;

	};

	class Resampler {
		const double ROLLOFF = 0.90;
		const double BETA = 6;

		void PopulateFilterCoeff();
		double ModBessel0th(const double x);
	public:
		static const u32 NUM_CROSSINGS = 5;
		static const u32 SAMPLES_PER_CROSSING = 65536;

		std::vector<std::vector<double> > m_lowpass_filter;
		std::vector<std::vector<double> > m_lowpass_delta;

		Resampler() {
			m_lowpass_filter.resize(SAMPLES_PER_CROSSING, std::vector<double>((NUM_CROSSINGS - 1), 0));
			m_lowpass_delta.resize(SAMPLES_PER_CROSSING, std::vector<double>((NUM_CROSSINGS - 1), 0));
			PopulateFilterCoeff();
		}

	};

	Resampler m_resampler;

	std::vector<std::vector<float> > m_sinc_table;   // [SINC_FSIZE][SINC_SIZE];

	MixerFifo m_dma_mixer;
	MixerFifo m_streaming_mixer;
	MixerFifo m_wiimote_speaker_mixer;
	u32 m_sample_rate;

	WaveFileWriter g_wave_writer_dtk;
	WaveFileWriter g_wave_writer_dsp;

	bool m_log_dtk_audio;
	bool m_log_dsp_audio;

	std::mutex m_cs_mixing;

	volatile float m_speed; // Current rate of the emulation (1.0 = 100% speed)

private:
	void dither1(float* l_sample, float* r_sample);
	void dither2(float* l_sample, float* r_sample);

	std::vector<float> m_output_buffer;
	std::vector<float> m_dither_buffer;

	// dither accumulators
	s32   l_rand1, l_rand2;
	s32   r_rand1, r_rand2;
	float l_error1, l_error2;
	float r_error1, r_error2;
	float m_phase;
	// Lipshitz's minimally audible FIR
	static const float SHAPED_BS[5];
};

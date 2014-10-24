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

#define SINC_FSIZE		65536	// sinc table granularity = 1 / SINC_FSIZE.
#define SINC_SIZE		(5 - 1) // see comment for populate_sinc_table()
#define DPPS_MASK		0xF1	   // tells DPPS to apply to all inputs, and store in 1st index
// Dither defines
#define DITHER_SHAPE	0.5f
#define DITHER_WORD		(0xFFFF)
#define DITHER_WIDTH	1.f / DITHER_WORD
#define DITHER_SIZE		DITHER_WIDTH / RAND_MAX
#define DITHER_OFFSET	DITHER_WIDTH * DITHER_SHAPE
#ifndef M_PI
#define M_PI  3.14159265358979323846
#endif

class CMixer {

public:
	CMixer(unsigned int BackendSampleRate)
		: m_dma_mixer(this, 32000)
		, m_streaming_mixer(this, 48000)
		, m_wiimote_speaker_mixer(this, 3000)
		, m_sampleRate(BackendSampleRate)
		, m_log_dtk_audio(0)
		, m_log_dsp_audio(0)
		, m_speed(0)
	{
		INFO_LOG(AUDIO_INTERFACE, "Mixer is initialized");
	}

	virtual ~CMixer() {}

	// Called from audio threads
	virtual unsigned int Mix(short* samples, unsigned int numSamples, bool consider_framelimit = true);

	// Called from main thread
	virtual void PushSamples(const short* samples, unsigned int num_samples);
	virtual void PushStreamingSamples(const short* samples, unsigned int num_samples);
	virtual void PushWiimoteSpeakerSamples(const short* samples, unsigned int num_samples, unsigned int sample_rate);
	unsigned int GetSampleRate() const { return m_sampleRate; }

	void SetDMAInputSampleRate(unsigned int rate);
	void SetStreamInputSampleRate(unsigned int rate);
	void SetStreamingVolume(unsigned int lvolume, unsigned int rvolume);
	void SetWiimoteSpeakerVolume(unsigned int lvolume, unsigned int rvolume);

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

	std::mutex& MixerCritical() { return m_csMixing; }

	float GetCurrentSpeed() const { return m_speed; }
	void UpdateSpeed(volatile float val) { m_speed = val; }

protected:
	class MixerFifo {
	public:
		MixerFifo(CMixer *mixer, unsigned sample_rate)
			: m_mixer(mixer)
			, m_input_sample_rate(sample_rate)
			, m_indexW(0)
			, m_indexR(0)
			, m_LVolume(256)
			, m_RVolume(256)
			, m_numLeftI(0.0f)
			, m_frac(0)
		{
			memset(m_buffer, 0, sizeof(m_buffer));
			srand((u32) time(NULL));
			memset(float_buffer, 0, sizeof(float_buffer));
			memset(m_sinc_table, 0, sizeof(m_sinc_table));
			populate_sinc_table();
		}
		void PushSamples(const short* samples, unsigned int num_samples);
		unsigned int Mix(short* samples, unsigned int numSamples, bool consider_framelimit = true);
		void SetInputSampleRate(unsigned int rate);
		void SetVolume(unsigned int lvolume, unsigned int rvolume);
		void populateFloats(u32 start, u32 stop);
		float twos2float(u16 s);
		s16 float2stwos(float f);
	private:
		float sinc_sinc(float x, float window_width);
		void populate_sinc_table();
		CMixer *m_mixer;
		unsigned m_input_sample_rate;
		float float_buffer[MAX_SAMPLES * 2];
		float m_sinc_table[SINC_FSIZE][SINC_SIZE];
		short m_buffer[MAX_SAMPLES * 2];
		volatile u32 m_indexW;
		volatile u32 m_indexR;
		u32 m_previousW;
		// Volume ranges from 0-256
		volatile s32 m_LVolume;
		volatile s32 m_RVolume;
		float m_numLeftI;
		float m_frac;
		int m_randL1, m_randL2, m_randR1, m_randR2;
		float m_errorL1, m_errorL2;
		float m_errorR1, m_errorR2;
	};
	MixerFifo m_dma_mixer;
	MixerFifo m_streaming_mixer;
	MixerFifo m_wiimote_speaker_mixer;
	unsigned int m_sampleRate;

	WaveFileWriter g_wave_writer_dtk;
	WaveFileWriter g_wave_writer_dsp;

	bool m_log_dtk_audio;
	bool m_log_dsp_audio;

	std::mutex m_csMixing;

	volatile float m_speed; // Current rate of the emulation (1.0 = 100% speed)
};

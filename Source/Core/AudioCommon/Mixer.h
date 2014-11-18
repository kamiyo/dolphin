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

class CMixer {

public:
	CMixer(u32 BackendSampleRate)
		: m_dma_mixer(this, 32000)
		, m_streaming_mixer(this, 48000)
		, m_wiimote_speaker_mixer(this, 3000)
		, m_sample_rate(BackendSampleRate)
		, m_log_dtk_audio(0)
		, m_log_dsp_audio(0)
		, m_speed(0)
	{
		INFO_LOG(AUDIO_INTERFACE, "Mixer is initialized");
	}

	virtual ~CMixer() {}

	// Called from audio threads
	virtual u32 Mix(s16* samples, u32 numSamples, bool consider_framelimit = true);

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

protected:
	class MixerFifo {
	public:
		MixerFifo(CMixer* mixer, u32 sample_rate)
			: m_mixer(mixer)
			, m_input_sample_rate(sample_rate)
			, m_write_index(0)
			, m_read_index(0)
			, m_LVolume(256)
			, m_RVolume(256)
			, m_num_left_i(0.0f)
			, m_fraction(0)
		{
			memset(m_buffer, 0, sizeof(m_buffer));
			srand((u32) time(NULL));
			memset(m_float_buffer, 0, sizeof(m_float_buffer));
			memset(m_sinc_table, 0, sizeof(m_sinc_table));
			PopulateSincTable();
		}
		void PushSamples(const s16* samples, u32 num_samples);
		u32  Mix(s16* samples, u32 numSamples, bool consider_framelimit = true);
		void SetInputSampleRate(u32 rate);
		void SetVolume(u32 lvolume, u32 rvolume);
		void PopulateFloats(u32 start, u32 stop);

	private:
		void     PopulateSincTable();
		CMixer*  m_mixer;
		u32      m_input_sample_rate;
		s16      m_buffer[MAX_SAMPLES * 2];
		float    m_sinc_table[SINC_FSIZE][SINC_SIZE];
		float    m_float_buffer[MAX_SAMPLES * 2];
		
		volatile u32 m_write_index;
		volatile u32 m_read_index;
		// Volume ranges from 0-256
		volatile u32 m_LVolume;
		volatile u32 m_RVolume;

		float m_num_left_i;
		float m_fraction;
		s32   m_randL1, m_randL2, m_randR1, m_randR2;
		float m_errorL1, m_errorL2;
		float m_errorR1, m_errorR2;
	};

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
};

// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2
// Refer to the license.txt file included.

// ---------------------------------------------------------------------------------
// Class: WaveFileWriter
// Description: Simple utility class to make it easy to write long 16-bit stereo
// audio streams to disk.
// Use Start() to start recording to a file, and AddStereoSamples to add wave data.
// The float variant will convert from -1.0-1.0 range and clamp.
// Alternatively, AddSamplesBE for big endian wave data.
// If Stop is not called when it destructs, the destructor will call Stop().
// ---------------------------------------------------------------------------------

#pragma once

#include <string>
#include "Common/Common.h"
#include "Common/FileUtil.h"

class WaveFileWriter : NonCopyable
{
public:
	WaveFileWriter();
	~WaveFileWriter();

	bool Start(const std::string& filename, unsigned int HLESampleRate);
	void Stop();

	void SetSkipSilence(bool skip) { skip_silence = skip; }

	void AddStereoSamples(const short *sample_data, u32 count);
	void AddStereoSamplesBE(const short *sample_data, u32 count);  // big endian
	u32 GetAudioSize() const { return audio_size; }

private:
	File::IOFile file;
	bool skip_silence;
	u32 audio_size;
	short* conv_buffer;
	void Write(u32 value);
	void Write4(const char* ptr);
};

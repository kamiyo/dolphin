// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2
// Refer to the license.txt file included.

#pragma once

#include <thread>

#include "AudioCommon/SoundStream.h"
#include "Common/Event.h"

class OpenSLESStream final : public SoundStream
{
#ifdef ANDROID
public:
	OpenSLESStream(CMixer *mixer)
		: SoundStream(mixer)
	{
	}

	virtual ~OpenSLESStream()
	{
	}

	virtual bool Start();
	virtual void Stop();
	static bool isValid() { return true; }

private:
	std::thread thread;
	Common::Event soundSyncEvent;
#else
public:
	OpenSLESStream(CMixer *mixer)
		: SoundStream(mixer)
	{
	}
#endif // HAVE_OPENSL
};

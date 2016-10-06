// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#include "Common/FileUtil.h"
#include "Common/Logging/Log.h"
#include "WindowedSincFilter.h"

static constexpr double BESSEL_EPSILON = 1e-21;
static constexpr double PI = 3.14159265358979323846;

double WindowedSincFilter::ModBesselZeroth(const double x) const
{
	double sum = 1.0;
	s32 factorial_store = 1;
	double half_x = x / 2.0;
	double previous = 1.0;
	do {
		double temp = half_x / (double)factorial_store;
		temp *= temp;
		previous *= temp;
		sum += previous;
		factorial_store++;
	} while (previous >= BESSEL_EPSILON * sum);
	return sum;
}

void WindowedSincFilter::PopulateFilterDeltas()
{
	for (u32 i = 0; i < m_wing_size - 1; ++i)
	{
		m_deltas[i] = m_coeffs[i + 1] - m_coeffs[i];
	}
	m_deltas[m_wing_size - 1] = -m_coeffs[m_wing_size - 1];
}

void WindowedSincFilter::PopulateFilterCoeffs()
{
	m_coeffs[0] = (float)(2 * m_cutoff_cycle);
	double inv_I0_beta = 1.0 / ModBesselZeroth(m_kaiser_beta);
	double inv_size = 1.0 / (m_wing_size - 1);
	for (u32 i = 1; i < m_wing_size; ++i)
	{
		double offset = PI * (double)i / (double)m_samples_per_crossing;
		double sinc = sin(offset * 2 * m_cutoff_cycle) / offset;
		double radicand = (double)i * inv_size;
		radicand = 1.0 - radicand * radicand;
		radicand = (radicand < 0.0) ? 0.0 : radicand;
		m_coeffs[i] = (float)(sinc * ModBesselZeroth(m_kaiser_beta * sqrt(radicand)) * inv_I0_beta);
	}

	double dc_gain = 0;
	for (u32 i = m_samples_per_crossing; i < m_wing_size; i += m_samples_per_crossing)
	{
		dc_gain += m_coeffs[i];
	}
	dc_gain *= 2;
	dc_gain += m_coeffs[0];
	double inv_dc_gain = 1.0 / dc_gain;

	for (u32 i = 0; i < m_wing_size; ++i)
	{
		m_coeffs[i] = (float)(m_coeffs[i] * inv_dc_gain);
	}
}

void WindowedSincFilter::CheckFilterCache()
{
	const std::string audio_cache_path = File::GetUserPath(D_CACHE_IDX) + "Audio/";

	if (!File::IsDirectory(audio_cache_path))
	{
		File::SetCurrentDir(File::GetUserPath(D_CACHE_IDX));

		if (!File::CreateDir("Audio"))
		{
			WARN_LOG(AUDIO, "failed to create audio cache folder: %s", audio_cache_path.c_str());
		}

		File::SetCurrentDir(File::GetExeDirectory());
	}

	std::stringstream filter_specs;
	filter_specs << m_num_crossings << "-"
		<< m_samples_per_crossing << "-"
		<< m_kaiser_beta << "-"
		<< m_cutoff_cycle;

	std::string filter_filename = audio_cache_path + "filter-" + filter_specs.str() + ".cache";
	std::string deltas_filename = audio_cache_path + "deltas-" + filter_specs.str() + ".cache";

	bool create_coeffs = true;
	bool create_deltas = true;

	if (ReadFilterFromFile(filter_filename, m_coeffs))
	{
		INFO_LOG(AUDIO_INTERFACE, "filter successfully retrieved from cache: %s", filter_filename.c_str());
		create_coeffs = false;

		if (ReadFilterFromFile(deltas_filename, m_deltas))
		{
			create_deltas = false;
		}
	}
	else {
		INFO_LOG(AUDIO_INTERFACE, "filter not retrieved from cache: %s", filter_filename.c_str());
	}

	if (create_coeffs)
	{
		PopulateFilterCoeffs();

		if (!WriteFilterToFile(filter_filename, m_coeffs))
		{
			WARN_LOG(AUDIO_INTERFACE, "did not successfully store filter to cache: %s", filter_filename.c_str());
		}
	}

	if (create_deltas)
	{
		PopulateFilterDeltas();

		if (!WriteFilterToFile(deltas_filename, m_deltas))
		{
			WARN_LOG(AUDIO_INTERFACE, "did not successfully store deltas to cache: %s", deltas_filename.c_str());
		}
	}
}

bool WindowedSincFilter::ReadFilterFromFile(const std::string &filename, float* filter)
{
	File::IOFile cache(filename, "rb");
	u32 cache_count = (u32)cache.GetSize() / sizeof(float);
	if (cache_count != m_wing_size)
	{
		cache.Close();
		return false;
	}
	bool success = cache.ReadArray<float>(filter, m_wing_size);
	cache.Close();
	return success;
}

bool WindowedSincFilter::WriteFilterToFile(const std::string &filename, float* filter)
{
	File::IOFile cache(filename, "wb");
	bool success = cache.WriteArray<float>(filter, m_wing_size);
	success = success && cache.Flush() && cache.Close();
	return success;
}
// Copyright 2016 Dolphin Emulator Project
// Licensed under GPLv2+
// Refer to the license.txt file included.

#pragma once

#include <string>

#include "Common/CommonTypes.h"

class WindowedSincFilter
{
public:
	// Default Parameters changeable
	WindowedSincFilter(u32 num_crossings = 13,	u32 samples_per_crossing = 512, float cutoff_cycle = 0.5, float beta = 7.0)
		: m_num_crossings(num_crossings)
		, m_samples_per_crossing(samples_per_crossing)
		, m_wing_size(samples_per_crossing * (num_crossings - 1) / 2)
		, m_cutoff_cycle(cutoff_cycle)
		, m_kaiser_beta(beta)
	{
		m_coeffs = (float*)malloc(m_wing_size * sizeof(float));
		m_deltas = (float*)malloc(m_wing_size * sizeof(float));
		CheckFilterCache();
	}

	~WindowedSincFilter()
	{
		free(m_coeffs);
		free(m_deltas);
		m_coeffs = m_deltas = nullptr;
	}

	float* m_coeffs;
	float* m_deltas;
	const u32 m_num_crossings;
	const u32 m_samples_per_crossing;
	const u32 m_wing_size;

private:
	void PopulateFilterCoeffs();
	void PopulateFilterDeltas();
	void CheckFilterCache();
	bool ReadFilterFromFile(const std::string &filename, float* filter);
	bool WriteFilterToFile(const std::string &filename, float* filter);
	double ModBesselZeroth(const double x) const;

	const float m_kaiser_beta;
	const float m_cutoff_cycle;
};


/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"
#include "MPIFrequency.hpp"

#include <cmath>

#include <set>
#include <string>

#include "geopm_hash.h"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"

namespace geopm
{
    MPIFrequency::MPIFrequency(PlatformIO &platform_io,
                               double freq_min, double freq_sticker,
                               double freq_max, double freq_step)
        : m_platform_io(platform_io)
        , m_freq_min(freq_min)
        , m_freq_sticker(freq_sticker)
        , m_freq_max(freq_max)
        , m_freq_step(freq_step)
    {

        // regions that should get near min.
        // system min doesn't seem to work even for MPI_Barrier.
        double system_min = m_platform_io.read_signal("FREQUENCY_MIN", GEOPM_DOMAIN_BOARD, 0);
        if (m_freq_min < system_min + 3*m_freq_step) {
            m_freq_min = system_min + 3*m_freq_step;
        }
        /// @todo: from function
        std::set<std::string> supported_names = {
            "MPI_Barrier",
            "MPI_Allreduce",
            "MPI_Send"
        };

        // default is max.
        // @todo: check that this is still >= m_freq_min
        for (const auto &name : supported_names) {
            uint64_t hash = geopm_crc32_str(name.c_str());
            m_region_freq[hash] = m_freq_max;
        }

        std::set<std::string> insensitive_regions = {
            "MPI_Barrier"
        };
        for (const auto &name : insensitive_regions) {
            uint64_t hash = geopm_crc32_str(name.c_str());
            m_region_freq[hash] = m_freq_min;
        }

        // regions that should get frequency a few steps down from max.
        // @todo check that this is still above m_freq_min
        std::set<std::string> middle_regions = {
            "MPI_Allreduce"
        };
        for (const auto &name : middle_regions) {
            uint64_t hash = geopm_crc32_str(name.c_str());
            m_region_freq[hash] = m_freq_sticker - 2*m_freq_step;
        }

    }

    double MPIFrequency::best_frequency(uint64_t region_hash) const
    {
        double result = NAN;
        if (is_supported_mpi_region(region_hash)) {
            result = m_region_freq.at(region_hash);
        }
        return result;
    }

    bool MPIFrequency::is_supported_mpi_region(uint64_t region_hash) const
    {
        return m_region_freq.find(region_hash) != m_region_freq.end();
    }
}

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

#ifndef MPIFREQUENCY_HPP_INCLUDE
#define MPIFREQUENCY_HPP_INCLUDE

#include <cstdint>

#include <map>

namespace geopm
{
    class PlatformIO;

    class MPIFrequency
    {
        public:
            /// @todo: probably want min/max to be inputs to
            ///        best_frequency to support dynamic policies.
            ///        ctor should still look at system limits to
            ///        determine bounds.  change map values from
            ///        frequency to some kind of enum.  alternatively,
            ///        could have a setter for min/max when it changes
            ///        that updates the whole region map with new
            ///        values.
            MPIFrequency(PlatformIO &platform_io,
                         double freq_min, double freq_sticker,
                         double freq_max, double freq_step);
            virtual ~MPIFrequency() = default;

            /// @brief Returns a guess of the best frequency for the
            ///        given region hash.  If the region hash is not
            ///        an MPI function supported by GEOPM, it returns NAN.
            /// @todo: input min/max/sticker here?  other input like
            ///        number of ranks?
            double best_frequency(uint64_t region_hash) const;

            /// @brief Returns whether the given region hash is
            ///        supported by geopm.
            /// @todo: this might belong somewhere else.
            bool is_supported_mpi_region(uint64_t region_hash) const;

        private:
            PlatformIO &m_platform_io;
            double m_freq_min;
            double m_freq_sticker;
            double m_freq_max;
            double m_freq_step;
            std::map<uint64_t, double> m_region_freq;
    };

}

#endif

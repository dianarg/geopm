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

#include <memory>

#include "gtest/gtest.h"

#include "MPIFrequency.hpp"
#include "geopm_hash.h"
#include "MockPlatformIO.hpp"

using geopm::MPIFrequency;
using testing::Return;
using testing::_;
using testing::AtLeast;

// dummy
std::set<std::string> all_mpi_names(void) {
    std::set<std::string> result = {
        "MPI_Barrier",
        "MPI_Allreduce",
        "MPI_Send"
    };
    return result;
}

class MPIFrequencyTest : public testing::Test
{
    protected:
        //MPIFrequencyTest();
        void SetUp(void);
        MockPlatformIO m_pio;
        double m_freq_min;
        double m_freq_max;
        double m_freq_sticker;
        double m_freq_step;
        std::shared_ptr<MPIFrequency> m_mpi_freq;
};


void MPIFrequencyTest::SetUp()
{
    m_freq_min = 1.4e9;
    m_freq_sticker = 1.6e9;
    m_freq_max = 1.8e9;
    m_freq_step = 1e8;

    ON_CALL(m_pio, read_signal("FREQUENCY_MIN", _, _))
        .WillByDefault(Return(1.0e9));

    EXPECT_CALL(m_pio, read_signal("FREQUENCY_MIN", _, _));
    m_mpi_freq = std::make_shared<MPIFrequency>(m_pio, m_freq_min, m_freq_sticker, m_freq_max, m_freq_step);
}

TEST_F(MPIFrequencyTest, supported_functions)
{
    // test that all supported functions return true and have a
    // non-NAN frequency assignment.
    for (const auto &name : all_mpi_names()) {
        uint64_t hash = geopm_crc32_str(name.c_str());
        EXPECT_TRUE(m_mpi_freq->is_supported_mpi_region(hash));
        double freq = m_mpi_freq->best_frequency(hash);
        EXPECT_FALSE(std::isnan(freq));
        EXPECT_LE(m_freq_min, freq);
        EXPECT_GE(m_freq_max, freq);
    }

    // unsupported function
    uint64_t hash = geopm_crc32_str("MPI_Invalid_func");
    EXPECT_FALSE(m_mpi_freq->is_supported_mpi_region(hash));
    double freq = m_mpi_freq->best_frequency(hash);
    EXPECT_TRUE(std::isnan(freq));
}

TEST_F(MPIFrequencyTest, frequency)
{
    uint64_t hash = 0;
    // MPI_Barrier gets min
    hash = geopm_crc32_str("MPI_Barrier");
    EXPECT_EQ(m_freq_min, m_mpi_freq->best_frequency(hash));

    // reduce gets sticker - 1 pstate
    hash = geopm_crc32_str("MPI_Allreduce");
    EXPECT_EQ(m_freq_sticker - 2*m_freq_step, m_mpi_freq->best_frequency(hash));

    // MPI_Send gets max
    hash = geopm_crc32_str("MPI_Send");
}

TEST_F(MPIFrequencyTest, system_min)
{
    EXPECT_CALL(m_pio, read_signal("FREQUENCY_MIN", _, _));

    // test that min must be at least 3 pstates above system min
    MPIFrequency mpi_freq(m_pio, 1.0e9, m_freq_sticker, m_freq_max, m_freq_step);
    uint64_t hash = 0;
    // MPI_Barrier gets min
    hash = geopm_crc32_str("MPI_Barrier");
    EXPECT_EQ(1.3e9, mpi_freq.best_frequency(hash));
}

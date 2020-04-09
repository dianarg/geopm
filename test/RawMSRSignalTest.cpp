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
#include "gmock/gmock.h"

#include "MockMSRIO.hpp"
#include "RawMSRSignal.hpp"
#include "geopm_hash.h"
#include "geopm_test.hpp"

using geopm::RawMSRSignal;
using testing::Return;

TEST(RawMSRSignalTest, read)
{
    std::shared_ptr<MockMSRIO> m_msrio = std::make_shared<MockMSRIO>();
    int cpu = 10;
    uint64_t offset = 0xABC;
    RawMSRSignal sig {m_msrio, cpu, offset, 3};
    uint64_t expected = 0x456;
    EXPECT_CALL(*m_msrio, read_msr(cpu, offset))
        .WillOnce(Return(expected));
    double result = sig.read();
    EXPECT_EQ(expected, geopm_signal_to_field(result));
}

TEST(RawMSRSignalTest, read_batch)
{
    std::shared_ptr<MockMSRIO> m_msrio = std::make_shared<MockMSRIO>();
    int cpu = 10;
    uint64_t offset = 0xABC;
    RawMSRSignal sig {m_msrio, cpu, offset, 2};
    uint64_t mapped_mem = 0;
    EXPECT_CALL(*m_msrio, add_read(cpu, offset))
        .WillOnce(Return(&mapped_mem));
    sig.setup_batch();

    // mock read_batch by updating raw memory
    uint64_t expected = 0x456;
    mapped_mem = expected;

    double result = sig.sample();
    EXPECT_EQ(expected, geopm_signal_to_field(result));
}

TEST(RawMSRSignalTest, const_methods)
{
    std::shared_ptr<MockMSRIO> m_msrio = std::make_shared<MockMSRIO>();
    int cpu = 10;
    uint64_t offset = 0xABC;
    int domain = 5;
    RawMSRSignal sig {m_msrio, cpu, offset, domain};
    EXPECT_EQ(domain, sig.domain());
    auto func = sig.format_function();
    EXPECT_TRUE(is_format_raw64(func));  // All raw MSRs are printed as 64-bit hex
}

TEST(RawMSRSignalTest, errors)
{
#ifdef GEOPM_DEBUG
    // cannot construct with null MSRIO
    GEOPM_EXPECT_THROW_MESSAGE(RawMSRSignal(0, nullptr, 0, 0),
                               GEOPM_ERROR_LOGIC, "no valid MSRIO");
#endif
}

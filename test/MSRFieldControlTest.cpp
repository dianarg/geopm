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

#include <memory>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "MSRFieldControl.hpp"
#include "MSR.hpp"
#include "Helper.hpp"
#include "MockMSRIO.hpp"
#include "geopm_test.hpp"

using geopm::MSRFieldControl;
using geopm::MSR;
using testing::Return;
using testing::_;

class MSRFieldControlTest : public ::testing::Test
{
    protected:
        void SetUp();

        std::shared_ptr<MockMSRIO> m_msrio;
        int m_cpu;
        uint64_t m_offset;
        int m_begin_bit;
        int m_end_bit;
        uint64_t m_mask;
        int m_idx;
};

void MSRFieldControlTest::SetUp()
{
    m_msrio = std::make_shared<MockMSRIO>();
    m_cpu = 1;
    m_offset = 0xABC;
    m_begin_bit = 16;
    m_end_bit = 23;
    m_mask = 0xFF0000;
    m_idx = 42;
}

TEST_F(MSRFieldControlTest, write_scale)
{
    double scalar = 1.5;
    auto ctl = geopm::make_unique<MSRFieldControl>(m_msrio, m_cpu, m_offset,
                                                   m_begin_bit, m_end_bit,
                                                   MSR::M_FUNCTION_SCALE, scalar);

    double value = 150;
    EXPECT_CALL(*m_msrio, write_msr(m_cpu, m_offset, 0x640000, m_mask));
    ctl->write(value);
}

TEST_F(MSRFieldControlTest, write_batch_scale)
{
    double scalar = 1.5;
    auto ctl = geopm::make_unique<MSRFieldControl>(m_msrio, m_cpu, m_offset,
                                                   m_begin_bit, m_end_bit,
                                                   MSR::M_FUNCTION_SCALE, scalar);

    double value = 150;
    EXPECT_CALL(*m_msrio, add_write(m_cpu, m_offset))
        .WillOnce(Return(m_idx));
    ctl->setup_batch();
    EXPECT_CALL(*m_msrio, adjust(m_idx, 0x640000, m_mask));
    ctl->adjust(value);


}



TEST_F(MSRFieldControlTest, write_log_half)
{
    double scalar = 1.0;
    auto ctl = geopm::make_unique<MSRFieldControl>(m_msrio, m_cpu, m_offset,
                                                   m_begin_bit, m_end_bit,
                                                   MSR::M_FUNCTION_LOG_HALF,
                                                   scalar);

    double value = 0.25;
    EXPECT_CALL(*m_msrio, write_msr(m_cpu, m_offset, 0x020000, m_mask));
    ctl->write(value);
}

TEST_F(MSRFieldControlTest, write_batch_log_half)
{
    double scalar = 1.0;
    auto ctl = geopm::make_unique<MSRFieldControl>(m_msrio, m_cpu, m_offset,
                                                   m_begin_bit, m_end_bit,
                                                   MSR::M_FUNCTION_LOG_HALF,
                                                   scalar);
    double value = 0.25;
    EXPECT_CALL(*m_msrio, add_write(m_cpu, m_offset))
        .WillOnce(Return(m_idx));
    ctl->setup_batch();
    EXPECT_CALL(*m_msrio, adjust(m_idx, 0x020000, m_mask));
    ctl->adjust(value);
}

#if 0

TEST_F(MSRFieldSignalTest, read_7_bit_float)
{
    double scalar = 3.0;
    auto ctl = geopm::make_unique<MSRFieldControl>(m_msrio, m_cpu, m_offset,
                                                   m_begin_bit, m_end_bit,
                                                   MSR::M_FUNCTION_7_BIT_FLOAT,
                                                   scalar);
    uint64_t raw_val = 0xF1418321;  // field is 0x41
    EXPECT_CALL(*m_raw, read())
        .WillOnce(Return(geopm_field_to_signal(raw_val)));
    double expected = 9.0;
    double result = sig->read();
    EXPECT_EQ(expected, result);
}

TEST_F(MSRFieldSignalTest, read_batch_7_bit_float)
{
    double scalar = 3.0;
    auto sig = geopm::make_unique<MSRFieldSignal>(m_raw, m_start, m_end,
                                                  MSR::M_FUNCTION_7_BIT_FLOAT,
                                                  scalar);
    EXPECT_CALL(*m_raw, setup_batch());
    sig->setup_batch();
    uint64_t raw_val = 0xF1418321;  // field is 0x41
    EXPECT_CALL(*m_raw, sample())
        .WillOnce(Return(geopm_field_to_signal(raw_val)));
    double expected = 9.0;
    double result = sig->sample();
    EXPECT_EQ(expected, result);

}

TEST_F(MSRFieldSignalTest, read_overflow)
{
    std::unique_ptr<Signal> sig = geopm::make_unique<MSRFieldSignal>(m_raw, 0, 3,
                                                                     MSR::M_FUNCTION_OVERFLOW, 1.0);
    double result = NAN, expected = NAN;
    // no overflow for any sequence of values
    expected = 5.0;
    EXPECT_CALL(*m_raw, read())
        .WillOnce(Return(geopm_field_to_signal(0x0005)));
    result = sig->read();
    EXPECT_DOUBLE_EQ(expected, result);

    expected = 4.0;
    EXPECT_CALL(*m_raw, read())
        .WillOnce(Return(geopm_field_to_signal(0x0004)));
    result = sig->read();
    EXPECT_DOUBLE_EQ(expected, result);

    expected = 10.0;
    EXPECT_CALL(*m_raw, read())
        .WillOnce(Return(geopm_field_to_signal(0x000A)));
    result = sig->read();
    EXPECT_DOUBLE_EQ(expected, result);

    expected = 1.0;
    EXPECT_CALL(*m_raw, read())
        .WillOnce(Return(geopm_field_to_signal(0x0001)));
    result = sig->read();
    EXPECT_DOUBLE_EQ(expected, result);
}

TEST_F(MSRFieldSignalTest, read_batch_overflow)
{
    auto sig = geopm::make_unique<MSRFieldSignal>(m_raw, 0, 3,
                                                  MSR::M_FUNCTION_OVERFLOW, 1.0);
    EXPECT_CALL(*m_raw, setup_batch());
    sig->setup_batch();
    double result = NAN, expected = NAN;
    // no overflow
    expected = 5.0;
    EXPECT_CALL(*m_raw, sample())
        .WillOnce(Return(geopm_field_to_signal(0x0005)));
    result = sig->sample();
    EXPECT_DOUBLE_EQ(expected, result);
    // one overflow
    expected = 20.0;  // 4 + 16
    EXPECT_CALL(*m_raw, sample())
        .WillOnce(Return(geopm_field_to_signal(0x0004)));
    result = sig->sample();
    EXPECT_DOUBLE_EQ(expected, result);
    // still one overflow
    expected = 26.0;  // 10 + 16
    EXPECT_CALL(*m_raw, sample())
        .WillOnce(Return(geopm_field_to_signal(0x000A)));
    result = sig->sample();
    EXPECT_DOUBLE_EQ(expected, result);
    // multiple overflow
    expected = 33.0;  // 1 + 16 + 16
    EXPECT_CALL(*m_raw, sample())
        .WillOnce(Return(geopm_field_to_signal(0x0001)));
    result = sig->sample();
    EXPECT_DOUBLE_EQ(expected, result);
}

TEST_F(MSRFieldSignalTest, real_counter)
{
    // Test with real counter values
    auto sig = geopm::make_unique<MSRFieldSignal>(m_raw, 0, 47,
                                                  MSR::M_FUNCTION_OVERFLOW, 1.0);
    EXPECT_CALL(*m_raw, setup_batch());
    sig->setup_batch();

    uint64_t input_value = 0xFFFFFF27AAE8;
    EXPECT_CALL(*m_raw, sample())
        .WillOnce(Return(geopm_field_to_signal(input_value)));
    double of_value = sig->sample();
    EXPECT_DOUBLE_EQ((double)input_value, of_value);

    // Setup funky rollover
    input_value = 0xFFFF000DD5D0;
    uint64_t expected_value = input_value + (1ull << 48); // i.e. 0x1FFFF000DD5D0
    EXPECT_CALL(*m_raw, sample())
        .WillOnce(Return(geopm_field_to_signal(input_value)));
    of_value = sig->sample();
    EXPECT_DOUBLE_EQ((double)expected_value, of_value)
                     << "\nActual is : 0x" << std::hex << (uint64_t)of_value << std::endl
                     << "Expected is : 0x" << std::hex << expected_value << std::endl;
                }


TEST_F(MSRFieldSignalTest, errors)
{
    // logic errors in contructor because this class is internal to MSRIOGroup.
    // @todo: consider whether Signal should be public to help writers of IOGroups
#ifdef GEOPM_DEBUG
    // cannot construct with null underlying signal
    GEOPM_EXPECT_THROW_MESSAGE(MSRFieldSignal(nullptr, 0, 0,
                                              MSR::M_FUNCTION_SCALE, 1.0),
                               GEOPM_ERROR_LOGIC, "raw_msr cannot be null");

    // invalid number of bits
    GEOPM_EXPECT_THROW_MESSAGE(MSRFieldSignal(m_raw, 0, 63,
                                              MSR::M_FUNCTION_SCALE, 1.0),
                               GEOPM_ERROR_LOGIC, "64-bit fields are not supported");
    GEOPM_EXPECT_THROW_MESSAGE(MSRFieldSignal(m_raw, 4, 0,
                                              MSR::M_FUNCTION_SCALE, 1.0),
                               GEOPM_ERROR_LOGIC, "begin bit must be <= end bit");

    // invalid encode function
    GEOPM_EXPECT_THROW_MESSAGE(MSRFieldSignal(m_raw, 0, 0,
                                              99, 1.0),
                               GEOPM_ERROR_LOGIC, "invalid encoding function");
#endif

    // cannot call sample without batch setup
    auto sig = geopm::make_unique<MSRFieldSignal>(m_raw, m_start, m_end,
                                                  MSR::M_FUNCTION_SCALE, 1.0);
    GEOPM_EXPECT_THROW_MESSAGE(sig->sample(), GEOPM_ERROR_RUNTIME,
                               "setup_batch() must be called before sample()");
}

#endif // #if 0

TEST_F(MSRFieldControlTest, setup_batch)
{
    auto ctl = geopm::make_unique<MSRFieldControl>(m_msrio, m_cpu, m_offset,
                                                   m_begin_bit, m_end_bit,
                                                   MSR::M_FUNCTION_SCALE, 1.0);
    // setup batch can be called multiple times without further side effects
    EXPECT_CALL(*m_msrio, add_write(_, _)).Times(1);
    ctl->setup_batch();
    ctl->setup_batch();
}


TEST_F(MSRFieldControlTest, errors)
{
    // cannot constuct with null msrio
    GEOPM_EXPECT_THROW_MESSAGE(MSRFieldControl(nullptr, m_cpu, m_offset,
                                               m_begin_bit, m_end_bit,
                                               geopm::MSR::M_FUNCTION_SCALE, 1.0),
                               GEOPM_ERROR_INVALID, "null MSRIO");

    // cannot call adjust without setup batch
    auto ctl = geopm::make_unique<MSRFieldControl>(m_msrio, m_cpu, m_offset,
                                                   m_begin_bit, m_end_bit,
                                                   MSR::M_FUNCTION_SCALE, 1.0);
    GEOPM_EXPECT_THROW_MESSAGE(ctl->adjust(123), GEOPM_ERROR_RUNTIME,
                               "adjust() before setup_batch()");

    // invalid encode function
    GEOPM_EXPECT_THROW_MESSAGE(MSRFieldControl(m_msrio, m_cpu, m_offset,
                                               m_begin_bit, m_end_bit,
                                               99, 1.0),
                               GEOPM_ERROR_INVALID, "invalid encoding function");
}

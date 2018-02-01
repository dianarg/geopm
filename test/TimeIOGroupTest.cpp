/*
 * Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
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

#include "gtest/gtest.h"
#include "TimeIOGroup.hpp"
#include "Exception.hpp"
#include "geopm_time.h"

using geopm::TimeIOGroup;

class TimeIOGroupTest : public :: testing :: Test
{
    protected:
        TimeIOGroup m_group;
};

TEST_F(TimeIOGroupTest, is_valid)
{
    EXPECT_TRUE(m_group.is_valid_signal("TIME"));
    EXPECT_FALSE(m_group.is_valid_signal("INVALID"));
    EXPECT_FALSE(m_group.is_valid_control("TIME"));
    EXPECT_FALSE(m_group.is_valid_control("INVALID"));
}

TEST_F(TimeIOGroupTest, push)
{
    EXPECT_NO_THROW(m_group.push_signal("TIME", 0, 0));
    EXPECT_THROW(m_group.push_signal("INVALID", 0, 0), geopm::Exception);
    EXPECT_THROW(m_group.push_control("TIME", 0, 0), geopm::Exception);
    EXPECT_THROW(m_group.push_control("INVALID", 0, 0), geopm::Exception);
}

TEST_F(TimeIOGroupTest, sample)
{
    // Can't sample before we push a signal
    EXPECT_THROW(m_group.sample(0), geopm::Exception);
    // Calling read_batch with no signals pushed is okay
    EXPECT_NO_THROW(m_group.read_batch());
    // Push a signal and make sure the index comes back 0
    int signal_idx = m_group.push_signal("TIME", 0, 0);
    EXPECT_EQ(0, signal_idx);
    // Pushing time twice should result in the same signal index
    signal_idx = m_group.push_signal("TIME", 0, 0);
    EXPECT_EQ(0, signal_idx);
    // Can't sample prior to reading
    EXPECT_THROW(m_group.sample(signal_idx), geopm::Exception);
    // Make sure that calling sample twice without calling
    // read_batch() in between results in the same answer.
    m_group.read_batch();
    double time0 = m_group.sample(signal_idx);
    sleep(1);
    double time1 = m_group.sample(signal_idx);
    EXPECT_EQ(time0, time1);
    m_group.read_batch();
    time1 = m_group.sample(signal_idx);
    EXPECT_NE(time0, time1);
    // Check that a one second spin is recorded as one second long.
    struct geopm_time_s spin0;
    struct geopm_time_s spin1;
    m_group.read_batch();
    geopm_time(&spin0);
    do {
        geopm_time(&spin1);
    } while (geopm_time_diff(&spin0, &spin1) < 1.0);
    time0 = m_group.sample(signal_idx);
    m_group.read_batch();
    time1 = m_group.sample(signal_idx);
    EXPECT_NEAR(time1 - time0, 1.0, 0.001);
    // Check for throw if sample index is out of range
    EXPECT_THROW(m_group.sample(1), geopm::Exception);
}

TEST_F(TimeIOGroupTest, adjust)
{
    EXPECT_NO_THROW(m_group.write_batch());
    EXPECT_THROW(m_group.adjust(0, 0.0), geopm::Exception);
    EXPECT_THROW(m_group.write_control("TIME", 0, 0, 0.0), geopm::Exception);
}

TEST_F(TimeIOGroupTest, read_signal)
{
    // Check that a one second spin is recorded as one second long.
    struct geopm_time_s spin0;
    struct geopm_time_s spin1;
    double time0 = m_group.read_signal("TIME", 0, 0);
    geopm_time(&spin0);
    do {
        geopm_time(&spin1);
    } while (geopm_time_diff(&spin0, &spin1) < 1.0);
    double time1 = m_group.read_signal("TIME", 0, 0);
    EXPECT_NEAR(time1 - time0, 1.0, 0.001);
    EXPECT_THROW(m_group.read_signal("INVALID", 0, 0), geopm::Exception);    
}

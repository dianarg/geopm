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

#include <cstdint>
#include "gtest/gtest.h"
#include "geopm_test.hpp"

#include "EditDistEpochRecordFilter.hpp"

using geopm::ApplicationSampler;
using geopm::EditDistEpochRecordFilter;

class EditDistEpochRecordFilterTest : public ::testing::Test
{
    protected:
        void SetUp();
        std::vector<int> m_in_events;
        std::vector<int> m_out_events;
};

void EditDistEpochRecordFilterTest::SetUp()
{
    m_in_events = {
        ApplicationSampler::M_EVENT_HINT,
        ApplicationSampler::M_EVENT_REGION_ENTRY,
        ApplicationSampler::M_EVENT_REGION_EXIT,
        ApplicationSampler::M_EVENT_PROFILE,
        ApplicationSampler::M_EVENT_REPORT,
        ApplicationSampler::M_EVENT_CLAIM_CPU,
        ApplicationSampler::M_EVENT_RELEASE_CPU,
        ApplicationSampler::M_EVENT_NAME_KEY,
    };
    m_out_events = {
        ApplicationSampler::M_EVENT_EPOCH_COUNT,
    };
}

TEST_F(EditDistEpochRecordFilterTest, one_region_repeated)
{
    uint64_t hash = 0xAULL;
    ApplicationSampler::m_record_s record {
        0.0,
        0,
        ApplicationSampler::M_EVENT_REGION_ENTRY,
        hash
    };

    double time = 0.0;
    geopm::EditDistEpochRecordFilter ederf(16);
    for (uint64_t count = 0; count <= 2; ++count) {
        std::vector<ApplicationSampler::m_record_s> result = ederf.filter(record);
        ASSERT_EQ(1ULL, result.size());
        EXPECT_EQ(time, result[0].time);
        EXPECT_EQ(0, result[0].process);
        EXPECT_EQ(ApplicationSampler::M_EVENT_REGION_ENTRY, result[0].event);
        EXPECT_EQ(hash, result[0].signal);
        time += 1.0;
        record.time = time;
        record.event = ApplicationSampler::M_EVENT_REGION_EXIT;
        result = ederf.filter(record);
        ASSERT_EQ(1ULL, result.size());
        EXPECT_EQ(time, result[0].time);
        EXPECT_EQ(0, result[0].process);
        EXPECT_EQ(ApplicationSampler::M_EVENT_REGION_EXIT, result[0].event);
        EXPECT_EQ(hash, result[0].signal);
        time += 1.0;
    }

    for (uint64_t count = 1; count <= 10; ++count) {
        std::vector<ApplicationSampler::m_record_s> result = ederf.filter(record);
        ASSERT_EQ(2ULL, result.size());
        EXPECT_EQ(time, result[0].time);
        EXPECT_EQ(0, result[0].process);
        EXPECT_EQ(ApplicationSampler::M_EVENT_REGION_ENTRY, result[0].event);
        EXPECT_EQ(hash, result[0].signal);
        EXPECT_EQ(time, result[0].time);
        EXPECT_EQ(0, result[0].process);
        EXPECT_EQ(ApplicationSampler::M_EVENT_EPOCH_COUNT, result[0].event);
        EXPECT_EQ(count, result[0].signal);
        time += 1.0;
        record.time = time;
        record.event = ApplicationSampler::M_EVENT_REGION_EXIT;
        result = ederf.filter(record);
        ASSERT_EQ(1ULL, result.size());
        EXPECT_EQ(time, result[0].time);
        EXPECT_EQ(0, result[0].process);
        EXPECT_EQ(ApplicationSampler::M_EVENT_REGION_EXIT, result[0].event);
        EXPECT_EQ(hash, result[0].signal);
    }
}

TEST_F(EditDistEpochRecordFilterTest, filter_in)
{
    ApplicationSampler::m_record_s record {};
    geopm::EditDistEpochRecordFilter ederf(16);
    for (auto event : m_in_events) {
        record.event = event;
        std::vector<ApplicationSampler::m_record_s> result = ederf.filter(record);
        ASSERT_EQ(1ULL, result.size());
        EXPECT_EQ(0.0, result[0].time);
        EXPECT_EQ(0, result[0].process);
        EXPECT_EQ(event, result[0].event);
        EXPECT_EQ(0ULL, result[0].signal);
    }
}

TEST_F(EditDistEpochRecordFilterTest, filter_out)
{
    ApplicationSampler::m_record_s record {};
    std::vector<ApplicationSampler::m_record_s> result;
    geopm::EditDistEpochRecordFilter ederf(16);
    for (auto event : m_out_events) {
        record.event = event;
        result = ederf.filter(record);
        EXPECT_EQ(0ULL, result.size());
    }
}


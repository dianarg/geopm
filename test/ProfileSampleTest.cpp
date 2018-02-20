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

#include "geopm_time.h"
#include "ProfileSample.hpp"

class ProfileSampleTest : public ::testing::Test
{
    public:
        ProfileSampleTest();
    protected:
        std::vector<int> m_rank;
        geopm::ProfileSample m_profile_sample;
};

ProfileSampleTest::ProfileSampleTest()
    : m_profile_sample(m_rank)
{

}


TEST_F(ProfileSampleTest, cpu_rank)
{
    struct geopm_time_s time1{{0, 0}};
    auto progress1 = m_profile_sample.per_cpu_progress(time1);
    auto region1 = m_profile_sample.per_cpu_region_id();
    EXPECT_EQ(0u, progress1.size());
    EXPECT_EQ(0u, region1.size());

    //m_profile_sample.cpu_rank({23, 45});

    auto progress2 = m_profile_sample.per_cpu_progress(time1);
    auto region2 = m_profile_sample.per_cpu_region_id();
    EXPECT_EQ(2u, progress2.size());
    EXPECT_EQ(2u, region2.size());

    // TODO: duplicate cpus
    //m_profile_sample.cpu_rank({23, 45, 45, 23});

    auto progress3 = m_profile_sample.per_cpu_progress(time1);
    auto region3 = m_profile_sample.per_cpu_region_id();
    EXPECT_EQ(2u, progress3.size());
    EXPECT_EQ(2u, region3.size());

}

// TODO test list
// update changes result of per_cpu_region_id
// update changes result of per_cpu_progress
// progress of 100% changes region id to GEOPM_REGION_ID_UNMARKED

// look at sample regulator tests

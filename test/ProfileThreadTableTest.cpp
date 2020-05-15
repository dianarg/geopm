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

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "ProfileThreadTable.hpp"
#include "MockPlatformTopo.hpp"

using testing::Return;

class ProfileThreadTableTest : public ::testing::Test
{

};

TEST_F(ProfileThreadTableTest, something)
{

    int m_num_cpu = 4;
    MockPlatformTopo m_topo;
    EXPECT_CALL(m_topo, num_domain(GEOPM_DOMAIN_CPU)).WillOnce(Return(m_num_cpu));
    size_t buffer_size = 64 * m_num_cpu;
    void *buffer = malloc(buffer_size);
    geopm::ProfileThreadTableImp table(m_topo, buffer_size, buffer);
    std::vector<double> progress(m_num_cpu);

    EXPECT_EQ(m_num_cpu, table.num_cpu());

    // before init
    // todo: -1 or 0?
    // regions dont matter for entry, but cleanup on region exit
    table.dump(progress);
    EXPECT_EQ(-1, progress[0]);
    EXPECT_EQ(-1, progress[1]);

    for (int ii = 0; ii < m_num_cpu; ++ii) {
        table.init(ii, 4);
    }
    table.enable(true);

    // std::vector<int> post_cpu_idx {0, 1, 0, 2, 1};
    // size_t curr_post = 0;
    // EXPECT_CALL(cpu_idx_thing, cpu_idx()).Times(35)
    //     .WillRepeatedly(DoAll(Return(post_cpu_idx[curr_post]),
    //                           Invoke([]() { ++curr_post; })));
    // alternatively, one post and dump call per iteration of a loop

    table.dump(progress);
    EXPECT_EQ(0, progress[0]);
    EXPECT_EQ(0, progress[1]);
    table.post(0);
    table.dump(progress);
    EXPECT_EQ(0.25, progress[0]);
    EXPECT_EQ(0, progress[1]);
    table.post(1);
    table.post(0);
    table.dump(progress);
    EXPECT_EQ(0.5, progress[0]);
    EXPECT_EQ(0.25, progress[1]);

}

// todo: init numiter
// todo: init chunck size
// todo: init num work =unit.
// error: wrong size vector to dump
// error: buffer size too small

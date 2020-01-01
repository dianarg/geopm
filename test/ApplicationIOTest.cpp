/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
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
#include <set>
#include <list>
#include <string>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "ApplicationIO.hpp"
#include "Helper.hpp"
#include "MockProfileEventBuffer.hpp"
#include "MockProfileSampler.hpp"
#include "MockPlatformIO.hpp"
#include "MockPlatformTopo.hpp"

using geopm::ApplicationIO;
using geopm::ApplicationIOImp;
using geopm::PlatformTopo;
using testing::_;
using testing::Return;

class ApplicationIOTest : public ::testing::Test
{
    protected:
        void SetUp();
        size_t m_num_cpu_domain;
        size_t m_num_package_domain;
        size_t m_num_memory_domain;
        std::string m_shm_key = "test_shm";
        std::shared_ptr<MockProfileSampler> m_sampler;
        MockPlatformIO m_platform_io;
        MockPlatformTopo m_platform_topo;
        MockProfileEventBuffer m_profile_event_buffer;
        std::unique_ptr<ApplicationIO> m_app_io;
};

void ApplicationIOTest::SetUp()
{
    m_sampler = std::make_shared<MockProfileSampler>();

    EXPECT_CALL(*m_sampler, initialize());
    EXPECT_CALL(*m_sampler, rank_per_node());
    EXPECT_CALL(*m_sampler, capacity());
    m_num_cpu_domain = 4;
    m_num_package_domain = 1;
    m_num_memory_domain = 1;
    EXPECT_CALL(m_platform_topo, num_domain(GEOPM_DOMAIN_CPU))
        .WillOnce(Return(m_num_cpu_domain));
    EXPECT_CALL(m_platform_topo, num_domain(GEOPM_DOMAIN_PACKAGE))
        .WillOnce(Return(m_num_package_domain));
    EXPECT_CALL(m_platform_topo, num_domain(GEOPM_DOMAIN_BOARD_MEMORY))
        .WillOnce(Return(m_num_memory_domain));
    EXPECT_CALL(m_platform_io, read_signal("ENERGY_PACKAGE", GEOPM_DOMAIN_PACKAGE, _))
        .Times(m_num_package_domain)
        .WillRepeatedly(Return(122.0/m_num_package_domain));
    EXPECT_CALL(m_platform_io, read_signal("ENERGY_DRAM", GEOPM_DOMAIN_BOARD_MEMORY, _))
        .Times(m_num_memory_domain)
        .WillRepeatedly(Return(221.0/m_num_memory_domain));
    std::vector<int> ranks {1, 2, 3, 4};
    EXPECT_CALL(*m_sampler, cpu_rank()).WillOnce(Return(ranks));
    m_app_io = geopm::make_unique<ApplicationIOImp>(m_shm_key, m_sampler, m_platform_io, m_platform_topo, m_profile_event_buffer);
    m_app_io->connect();
}

TEST_F(ApplicationIOTest, passthrough)
{
    EXPECT_CALL(*m_sampler, do_shutdown()).WillOnce(Return(false));
    EXPECT_FALSE(m_app_io->do_shutdown());

    EXPECT_CALL(*m_sampler, report_name()).WillOnce(Return("my_report"));
    EXPECT_EQ("my_report", m_app_io->report_name());

    EXPECT_CALL(*m_sampler, profile_name()).WillOnce(Return("my_profile"));
    EXPECT_EQ("my_profile", m_app_io->profile_name());

    std::set<std::string> regions = {"region A", "region B"};
    EXPECT_CALL(*m_sampler, name_set()).WillOnce(Return(regions));
    EXPECT_EQ(regions, m_app_io->region_name_set());
}

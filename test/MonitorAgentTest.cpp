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
#include "gmock/gmock.h"

#include "MonitorAgent.hpp"
#include "MockPlatformIO.hpp"
#include "MockPlatformTopo.hpp"
#include "Helper.hpp"
#include "Agg.hpp"

using geopm::IPlatformTopo;
using geopm::IPlatformIO;
using geopm::MonitorAgent;
using ::testing::_;
using ::testing::Return;

class MonitorAgentTest : public ::testing::Test
{
    protected:
        enum signal_idx_e {
            M_OTHER,  // signal not used by this agent; index may not start at 0
            M_POWER_PACKAGE,
            M_FREQUENCY,
            M_INST_RETIRED,
        };
        MonitorAgentTest();
        void SetUp();
        MockPlatformIO m_platform_io;
        MockPlatformTopo m_platform_topo;
        IPlatformIO &m_plat_io_ref;
        IPlatformTopo &m_plat_topo_ref;
        std::unique_ptr<geopm::MonitorAgent> m_agent;
};

MonitorAgentTest::MonitorAgentTest()
    : m_plat_io_ref(m_platform_io)
    , m_plat_topo_ref(m_platform_topo)
{

}

void MonitorAgentTest::SetUp()
{
    // all signals are over entire board for now
    ON_CALL(m_platform_io, push_signal("POWER_PACKAGE", IPlatformTopo::M_DOMAIN_BOARD, 0))
        .WillByDefault(Return(M_POWER_PACKAGE));
    ON_CALL(m_platform_io, push_signal("FREQUENCY", IPlatformTopo::M_DOMAIN_BOARD, 0))
        .WillByDefault(Return(M_FREQUENCY));
    ON_CALL(m_platform_io, push_signal("INSTRUCTIONS_RETIRED", IPlatformTopo::M_DOMAIN_BOARD, 0))
        .WillByDefault(Return(M_INST_RETIRED));

    EXPECT_CALL(m_platform_io, push_signal("POWER_PACKAGE", _, _));
    EXPECT_CALL(m_platform_io, push_signal("FREQUENCY", _, _));
    EXPECT_CALL(m_platform_io, push_signal("INSTRUCTIONS_RETIRED", _, _));

    m_agent = geopm::make_unique<MonitorAgent>(m_plat_io_ref, m_plat_topo_ref);
}

TEST_F(MonitorAgentTest, fixed_signal_list)
{
    // default list we collect with this agent
    // if this list changes, update the mocked platform for this test
    std::vector<std::string> expected_signals = {"POWER_SUM", "FREQUENCY_SUM", "INST_RETIRED_SUM"};
    EXPECT_EQ(expected_signals, m_agent->sample_names());
}

TEST_F(MonitorAgentTest, sample_platform)
{
    std::vector<double> expected_value {456, 789, 123};
    EXPECT_CALL(m_platform_io, sample(M_POWER_PACKAGE))
        .WillOnce(Return(expected_value[0]));
    EXPECT_CALL(m_platform_io, sample(M_FREQUENCY))
        .WillOnce(Return(expected_value[1]));
    EXPECT_CALL(m_platform_io, sample(M_INST_RETIRED))
        .WillOnce(Return(expected_value[2]));

    std::vector<double> result(expected_value.size());
    m_agent->sample_platform(result);
    EXPECT_EQ(expected_value, result);
}

TEST_F(MonitorAgentTest, descend_nothing)
{
    std::vector<std::string> expected_policy_names = {"DEVIATION_PERCENT", "POWER_MEAN", "FREQ_MEAN", "INST_RETIRED_MEAN"};
    EXPECT_EQ(expected_policy_names, m_agent->policy_names());
}

TEST_F(MonitorAgentTest, ascend_aggregates_signals)
{
    std::vector<std::vector<double> > input = {
        {3, 8, 4},
        {4, 9, 7},
        {5, 10, 15}
    };
    // all are expected to be sum
    std::vector<double> expected = {12, 27, 26};
    std::vector<double> result(expected.size());;
    m_agent->ascend(input, result);

    EXPECT_EQ(expected, result);
}

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

#include <memory>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "MockPowerGovernor.hpp"
#include "MockPowerBalancerStep.hpp"
#include "MockPowerBalancer.hpp"
#include "MockPlatformIO.hpp"
#include "MockPlatformTopo.hpp"
#include "PowerBalancerAgent.hpp"
#include "Helper.hpp"

using geopm::PowerBalancerAgent;
using geopm::IPlatformTopo;
using ::testing::_;
using ::testing::ContainerEq;
using ::testing::Return;
using ::testing::Sequence;

class PowerBalancerAgentTest : public ::testing::Test
{
    protected:
        enum {
            M_SIGNAL_EPOCH_RUNTIME,
            M_SIGNAL_EPOCH_COUNT,
        };

        const int M_NUM_CTL_STEP = 6;///@todo?
        MockPlatformIO m_platform_io;
        MockPlatformTopo m_platform_topo;
        std::unique_ptr<MockPowerBalancerStep> m_power_step;
        std::unique_ptr<MockPowerGovernor> m_power_gov;
        std::unique_ptr<MockPowerBalancer> m_power_bal;
        std::unique_ptr<PowerBalancerAgent> m_agent;

        std::vector<int> m_fan_in = {2, 2};
        std::vector<int> m_level = {0, 1, 2};
};

#if 0
TEST_F(PowerBalancerAgentTest, XXXX_agent)
{
    bool is_root = true;
    for (const auto &level : m_level) {
        for (int ctl_step = 0; ctl_step < M_NUM_CTL_STEP; ++ctl_step) {
            m_power_step = std::unique_ptr<MockPowerBalancerStep>(new MockPowerBalancerStep());
            m_power_gov = std::unique_ptr<MockPowerGovernor>(new MockPowerGovernor());
            m_power_bal = std::unique_ptr<MockPowerBalancer>(new MockPowerBalancer());

            int num_children = 1;
            if (level == 0) {   /// leaf agent
                EXPECT_CALL(m_platform_io, push_signal("EPOCH_RUNTIME", IPlatformTopo::M_DOMAIN_BOARD, 0))
                    .WillOnce(Return(M_SIGNAL_EPOCH_RUNTIME));
                EXPECT_CALL(m_platform_io, push_signal("EPOCH_COUNT", IPlatformTopo::M_DOMAIN_BOARD, 0))
                    .WillOnce(Return(M_SIGNAL_EPOCH_COUNT));

                EXPECT_CALL(*m_power_gov, init_platform_io());
            }
            else {
                /// set negative expectations???  how pedantic are we going to be about this as a team?
                num_children = m_fan_in[level - 1];
            }

            m_agent = geopm::make_unique<PowerBalancerAgent>(m_platform_io, m_platform_topo,
                                                             std::move(m_power_step),
                                                             std::move(m_power_gov), std::move(m_power_bal));
            m_agent->init(level, m_fan_in, is_root);

            std::vector<double> in_policy = {NAN, NAN, NAN, NAN};
            std::vector<std::vector<double> > out_policy(num_children, {NAN, NAN, NAN, NAN});
            std::vector<std::vector<double> > exp_out_policy(num_children, {NAN, NAN, NAN, NAN});

            /// @todo
            //GEOPM_EXPECT_THROW_MESSAGE(m_agent->descend({}, {}), GEOPM_ERROR_LOGIC,
                    //"sample vectors not correctly sized.");
            m_agent->descend(in_policy, out_policy);
            m_agent->adjust_platform(in_policy);

            std::vector<std::vector<double> > in_sample(num_children, {NAN, NAN, NAN});
            std::vector<double> out_sample = {NAN, NAN, NAN};
            std::vector<double> exp_out_sample = {NAN, NAN, NAN};

            m_agent->sample_platform(out_sample);
            m_agent->ascend(in_sample, out_sample);
        }
    }
}
#endif

TEST_F(PowerBalancerAgentTest, tree_root_agent)
{
    const bool IS_ROOT = true;
    for (int ctl_step = 0; ctl_step < M_NUM_CTL_STEP; ++ctl_step) {
        m_power_step = std::unique_ptr<MockPowerBalancerStep>(new MockPowerBalancerStep());
        m_power_gov = std::unique_ptr<MockPowerGovernor>(new MockPowerGovernor());
        m_power_bal = std::unique_ptr<MockPowerBalancer>(new MockPowerBalancer());

        int level = 2;
        int num_children = m_fan_in[level - 1];
        bool exp_decscend_ret = false;
        bool exp_acscend_ret = false;

        std::vector<double> in_policy;
        std::vector<std::vector<double> > out_policy = std::vector<std::vector<double> >(num_children, {NAN, NAN, NAN, NAN});
        std::vector<std::vector<double> > exp_out_policy;

        std::vector<std::vector<double> > in_sample;
        std::vector<double> out_sample = {NAN, NAN, NAN};
        std::vector<double> exp_out_sample;

        switch (ctl_step) {
            case 0:
                in_policy = {300, NAN, NAN, NAN};
                exp_out_policy = std::vector<std::vector<double> >(num_children, {300, (double)ctl_step, 0.0, 0.0});

                in_sample = std::vector<std::vector<double> >(num_children, {(double)ctl_step, 22.0, 0.0});
                exp_out_sample = {(double)ctl_step, 22.0, 0.0};
                EXPECT_CALL(*m_power_step, step_count())
                    .Times(3)
                    .WillRepeatedly(Return(ctl_step));
                EXPECT_CALL(*m_power_step, is_send_down_limit())
                    .WillOnce(Return(true));
                EXPECT_CALL(*m_power_step, reset())
                    .Times(2)
                    .WillRepeatedly(Return(0));
                EXPECT_CALL(*m_power_step, is_step_complete())
                    .WillOnce(Return(false));
                EXPECT_CALL(*m_power_step, is_step_complete(true))
                    .Times(1);
                exp_decscend_ret = true;
                exp_acscend_ret = true;
                break;
        }

        m_agent = geopm::make_unique<PowerBalancerAgent>(m_platform_io, m_platform_topo,
                                                         std::move(m_power_step),
                                                         std::move(m_power_gov), std::move(m_power_bal));
        m_agent->init(level, m_fan_in, IS_ROOT);

        /// if policy_in[M_POLICY_POWER_CAP] != m_root_cap and else
        ///expect call reset, will once return 0
        bool desc_ret = m_agent->descend(in_policy, out_policy);
        EXPECT_EQ(exp_decscend_ret, desc_ret);
        EXPECT_THAT(exp_out_policy, ContainerEq(out_policy));

        ///expect call is_step_complete, will once return true
        bool ascend_ret = m_agent->ascend(in_sample, out_sample);
        EXPECT_EQ(exp_acscend_ret, ascend_ret);
        EXPECT_THAT(exp_out_sample, ContainerEq(out_sample));
        return;
    }
}

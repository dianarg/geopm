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

#include <cmath>

#include <vector>
#include <memory>
#include <map>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "EnergyEfficientAgent.hpp"
#include "geopm.h"
#include "MockPlatformIO.hpp"
#include "MockPlatformTopo.hpp"
#include "MockFrequencyGovernor.hpp"
#include "MockEnergyEfficientRegion.hpp"
#include "Helper.hpp"
#include "geopm_test.hpp"
#include "config.h"

using geopm::EnergyEfficientAgent;
using geopm::EnergyEfficientRegion;

using testing::Return;
using testing::_;
using testing::DoAll;
using testing::SetArgReferee;
using testing::AtLeast;

class EnergyEfficientAgentTest : public :: testing :: Test
{
    protected:
        void SetUp();
        MockPlatformIO m_platio;
        MockPlatformTopo m_topo;
        std::shared_ptr<MockFrequencyGovernor> m_gov;
        std::unique_ptr<EnergyEfficientAgent> m_agent0;
        std::unique_ptr<EnergyEfficientAgent> m_agent1;
        static constexpr int M_NUM_CHILDREN = 3;
        static constexpr int M_FREQ_DOMAIN = GEOPM_DOMAIN_CORE;
        static constexpr int M_NUM_FREQ_DOMAIN = 4;
        static constexpr double M_PERF_MARGIN = 0.10;
        // offsets for PlatformIO signal indices
        const int HASH_SIG = 1000;
        const int HINT_SIG = 2000;
        const int RUNTIME_SIG = 3000;
        const int COUNT_SIG = 4000;
        std::map<uint64_t, std::shared_ptr<MockEnergyEfficientRegion> > m_region_map;
        std::vector<uint64_t> m_region_hashes;
        void expect_aggregation(std::unique_ptr<EnergyEfficientAgent>& agent,
                                const std::vector<double> &expected_out,
                                const std::vector<std::vector<double> > &in_sample);
};

void EnergyEfficientAgentTest::SetUp()
{
    m_gov = std::make_shared<MockFrequencyGovernor>();

    ON_CALL(*m_gov, frequency_domain_type())
        .WillByDefault(Return(M_FREQ_DOMAIN));
    ON_CALL(m_topo, num_domain(M_FREQ_DOMAIN))
        .WillByDefault(Return(M_NUM_FREQ_DOMAIN));

    std::vector<int> fan_in {M_NUM_CHILDREN};
    m_region_map[0x12] = std::make_shared<MockEnergyEfficientRegion>();
    m_region_map[0x34] = std::make_shared<MockEnergyEfficientRegion>();
    m_region_map[0x56] = std::make_shared<MockEnergyEfficientRegion>();
    m_region_map[0x78] = std::make_shared<MockEnergyEfficientRegion>();
    std::transform(
        m_region_map.begin(), m_region_map.end(), std::back_inserter(m_region_hashes),
        [](decltype(m_region_map)::value_type entry) { return entry.first; });
    // expectations for constructor
    EXPECT_CALL(m_topo, num_domain(M_FREQ_DOMAIN)).Times(2);
    EXPECT_CALL(*m_gov, frequency_domain_type()).Times(2);
    for (int idx = 0; idx < M_NUM_FREQ_DOMAIN; ++idx) {
        EXPECT_CALL(m_platio, push_signal("REGION_HASH", M_FREQ_DOMAIN, idx))
            .WillOnce(Return(HASH_SIG + idx));
        EXPECT_CALL(m_platio, push_signal("REGION_HINT", M_FREQ_DOMAIN, idx))
            .WillOnce(Return(HINT_SIG + idx));
        EXPECT_CALL(m_platio, push_signal("REGION_RUNTIME", M_FREQ_DOMAIN, idx))
            .WillOnce(Return(RUNTIME_SIG + idx));
        EXPECT_CALL(m_platio, push_signal("REGION_COUNT", M_FREQ_DOMAIN, idx))
            .WillOnce(Return(COUNT_SIG + idx));
    }
    std::map<uint64_t, std::shared_ptr<EnergyEfficientRegion> > region_map;
    for (auto &kv : m_region_map) {
        region_map[kv.first] = kv.second;
    }
    m_agent0 = geopm::make_unique<EnergyEfficientAgent>(m_platio, m_topo, m_gov, region_map);
    m_agent1 = geopm::make_unique<EnergyEfficientAgent>(m_platio, m_topo, m_gov,
                                                        std::map<uint64_t, std::shared_ptr<EnergyEfficientRegion> >());

    // expectations for init
    EXPECT_CALL(*m_gov, init_platform_io()).Times(1);
    m_agent0->init(0, fan_in, false);
    m_agent1->init(1, fan_in, false);
}

// Set an exception that a given vector of samples is aggregated into a given
// output vector.
void EnergyEfficientAgentTest::expect_aggregation(
    std::unique_ptr<EnergyEfficientAgent> &agent, const std::vector<double> &expected_out,
    const std::vector<std::vector<double> > &in_sample)
{
    std::vector<double> out_sample(expected_out.size(), NAN);
    agent->aggregate_sample(in_sample, out_sample);
    EXPECT_THAT(out_sample, IsEqualToPolicy(expected_out));
}


TEST_F(EnergyEfficientAgentTest, validate_policy_default)
{
    // set up expectations for NAN policy: system min and max
    double sys_min = 1.0e9;
    double sys_max = 2.0e9;
    EXPECT_CALL(*m_gov, validate_policy(_, _))
        .WillOnce(DoAll(SetArgReferee<0>(sys_min),
                        SetArgReferee<1>(sys_max)));
    std::vector<double> in_policy {NAN, NAN, NAN};
    std::vector<double> expected {sys_min, sys_max, 0.10};
    m_agent0->validate_policy(in_policy);
    EXPECT_THAT(in_policy, IsEqualToPolicy(expected));
}

TEST_F(EnergyEfficientAgentTest, validate_policy_clamp)
{
    EXPECT_CALL(*m_gov, validate_policy(_, _));
    // validate policy does not do clamping for frequency
    std::vector<double> wide_policy {0.9e9, 2.1e9, 0.5};
    std::vector<double> in_policy = wide_policy;
    m_agent0->validate_policy(in_policy);
    EXPECT_THAT(in_policy, IsEqualToPolicy(wide_policy));
}

TEST_F(EnergyEfficientAgentTest, validate_policy_perf_margin)
{
    std::vector<double> in_policy {NAN, NAN, -0.2};
    EXPECT_THROW(m_agent0->validate_policy(in_policy), geopm::Exception);
    in_policy = {NAN, NAN, 1.2};
    EXPECT_THROW(m_agent0->validate_policy(in_policy), geopm::Exception);
}

TEST_F(EnergyEfficientAgentTest, validate_policy_variable_length)
{
    std::vector<double> in_policy {NAN, NAN, 0.2};
    std::vector<double> expected_policy(in_policy);

    m_agent0->validate_policy(in_policy);
    EXPECT_THAT(in_policy, IsEqualToPolicy(expected_policy));

    in_policy = {NAN, NAN, 0.2, NAN};
    EXPECT_THROW(m_agent0->validate_policy(in_policy), geopm::Exception)
        << "Incomplete hash/frequency pair";

    in_policy = {NAN, NAN, 0.2, NAN, NAN};
    expected_policy = in_policy;
    m_agent0->validate_policy(in_policy);
    EXPECT_THAT(in_policy, IsEqualToPolicy(expected_policy));

    in_policy = {NAN, NAN, 0.2, NAN, NAN, NAN};
    EXPECT_THROW(m_agent0->validate_policy(in_policy), geopm::Exception)
        << "Trailing NAN after an empty hash/frequency pair";

    in_policy = {NAN, NAN, 0.2, NAN, NAN, static_cast<double>(m_region_hashes[0]), NAN};
    expected_policy = in_policy;
    m_agent0->validate_policy(in_policy);
    EXPECT_THAT(in_policy, IsEqualToPolicy(expected_policy));

    in_policy = {NAN, NAN, 0.2, NAN, NAN, NAN, 0.6};
    EXPECT_THROW(m_agent0->validate_policy(in_policy), geopm::Exception)
        << "Attempt to map a frequency to a non-region";
}

TEST_F(EnergyEfficientAgentTest, split_policy_unchanged)
{
    double in_pol_min = 1.1e9;
    double in_pol_max = 2.1e9;
    std::vector<double> in_policy {in_pol_min, in_pol_max, M_PERF_MARGIN};
    std::vector<double> garbage {5.67, 8.90, 7.8};
    std::vector<std::vector<double> > out_policy {M_NUM_CHILDREN, garbage};

    EXPECT_CALL(*m_gov, set_frequency_bounds(in_pol_min, in_pol_max))
        .WillOnce(Return(false));
    m_agent1->split_policy(in_policy, out_policy);
    EXPECT_FALSE(m_agent1->do_send_policy());
    // out_policy will not be modified
    for (const auto &child_policy : out_policy) {
        EXPECT_EQ(garbage, child_policy);
    }
}

TEST_F(EnergyEfficientAgentTest, split_policy_changed)
{
    double in_pol_min = 1.1e9;
    double in_pol_max = 2.1e9;
    std::vector<double> in_policy {in_pol_min, in_pol_max, M_PERF_MARGIN};
    std::vector<double> garbage {5.67, 8.90, 7.9};
    std::vector<std::vector<double> > out_policy {M_NUM_CHILDREN, garbage};

    EXPECT_CALL(*m_gov, set_frequency_bounds(in_pol_min, in_pol_max))
        .WillOnce(Return(true));
    m_agent1->split_policy(in_policy, out_policy);
    EXPECT_TRUE(m_agent1->do_send_policy());
    for (const auto &child_policy : out_policy) {
        EXPECT_EQ(in_policy, child_policy);
    }
}

TEST_F(EnergyEfficientAgentTest, split_policy_errors)
{
#ifdef GEOPM_DEBUG
    std::vector<double> in_policy {1.2e9, 1.4e9, M_PERF_MARGIN};
    std::vector<std::vector<double> > out_policy {M_NUM_CHILDREN, in_policy};
    std::vector<double> bad_in {4, 4, 4, 4, 4, 4};
    std::vector<std::vector<double> > bad_out1 {8, in_policy};
    std::vector<std::vector<double> > bad_out2 {M_NUM_CHILDREN, bad_in};

    GEOPM_EXPECT_THROW_MESSAGE(m_agent1->split_policy(bad_in, out_policy), GEOPM_ERROR_LOGIC,
                               "in_policy vector not correctly sized");
    GEOPM_EXPECT_THROW_MESSAGE(m_agent1->split_policy(in_policy, bad_out1), GEOPM_ERROR_LOGIC,
                               "out_policy vector not correctly sized");
    GEOPM_EXPECT_THROW_MESSAGE(m_agent1->split_policy(in_policy, bad_out2), GEOPM_ERROR_LOGIC,
                               "child_policy vector not correctly sized");
#endif
}

TEST_F(EnergyEfficientAgentTest, aggregate_sample)
{
    expect_aggregation(
        m_agent1, { NAN, NAN, NAN, NAN },
        { { NAN, NAN, NAN, NAN }, { NAN, NAN, NAN, NAN }, { NAN, NAN, NAN, NAN } });
    EXPECT_FALSE(m_agent1->do_send_sample());

    expect_aggregation(m_agent1, { 0x1234, 1e9, NAN, NAN },
                       { { 0x1234, 1e9, NAN, NAN },
                         { 0x1234, 1e9, NAN, NAN },
                         { 0x2468, 2e9, 0x1234, 1e9 } });
    EXPECT_TRUE(m_agent1->do_send_sample());

    expect_aggregation(m_agent1, { 0x1234, 2e9, NAN, NAN },
                       { { 0x1234, 1e9, NAN, NAN },
                         { 0x1234, 2e9, NAN, NAN },
                         { 0x2468, 2e9, 0x1234, 1e9 } });
    EXPECT_TRUE(m_agent1->do_send_sample());

    expect_aggregation(m_agent1, { 0x1234, 2e9, NAN, NAN },
                       { { 0x1234, 1e9, NAN, NAN },
                         { 0x1234, 2e9, NAN, NAN },
                         { 0x2468, 2e9, 0x1234, 1e9 } });
    EXPECT_TRUE(m_agent1->do_send_sample());

    expect_aggregation(m_agent1, { 0x1234, 2e9, 0x2468, 3e9 },
                       { { 0x1234, 1e9, 0x2468, 3e9 },
                         { 0x1234, 2e9, 0x2468, 3e9 },
                         { 0x2468, 2e9, 0x1234, 1e9 } });
    EXPECT_TRUE(m_agent1->do_send_sample());
}

TEST_F(EnergyEfficientAgentTest, aggregate_sample_level0)
{
    std::vector<double> empty{ NAN, NAN };
    std::vector<double> out_sample{ NAN, NAN };
    std::vector<std::vector<double> > in_sample{ M_NUM_CHILDREN, empty };
#ifdef GEOPM_DEBUG
    EXPECT_THROW(m_agent0->aggregate_sample(in_sample, out_sample), geopm::Exception)
        << "aggregate_sample() shouldn't be called on level 0. On GEOPM_DEBUG "
           "builds, it should fail to indicate the bad call";
#else
    EXPECT_NO_THROW(m_agent0->aggregate_sample(in_sample, out_sample))
        << "aggregate_sample() shouldn't be called on level 0, but "
           "it should do nothing if that does happen";
#endif
    EXPECT_FALSE(m_agent0->do_send_sample());

    // no samples to aggregate
    EXPECT_THAT(out_sample, IsEqualToPolicy(empty));
}

TEST_F(EnergyEfficientAgentTest, do_write_batch)
{
    // pass through to FrequencyGovernor
    EXPECT_CALL(*m_gov, do_write_batch())
        .WillOnce(Return(true))
        .WillOnce(Return(false));
    EXPECT_TRUE(m_agent0->do_write_batch());
    EXPECT_FALSE(m_agent0->do_write_batch());
}

TEST_F(EnergyEfficientAgentTest, static_methods)
{
    EXPECT_EQ("energy_efficient", EnergyEfficientAgent::plugin_name());
    std::vector<std::string> pol_names {"FREQ_MIN", "FREQ_MAX", "PERF_MARGIN"};
    for (int i = 0; i < 32; ++i) {
        pol_names.emplace_back("HASH_" + std::to_string(i));
        pol_names.emplace_back("FREQ_" + std::to_string(i));
    }
    std::vector<std::string> sam_names {};
    for (int i = 0; i < 32; ++i) {
        sam_names.emplace_back("HASH_" + std::to_string(i));
        sam_names.emplace_back("FREQ_" + std::to_string(i));
    }
    EXPECT_EQ(pol_names, EnergyEfficientAgent::policy_names());
    EXPECT_EQ(sam_names, EnergyEfficientAgent::sample_names());
}

TEST_F(EnergyEfficientAgentTest, enforce_policy)
{
    const double limit = 1e9;
    const std::vector<double> policy{0, limit, 0.15};
    const std::vector<double> bad_policy{100, 200, 300, 400};

    EXPECT_CALL(m_platio, write_control("FREQUENCY", GEOPM_DOMAIN_BOARD, 0, limit));

    m_agent0->enforce_policy(policy);

    EXPECT_THROW(m_agent0->enforce_policy(bad_policy), geopm::Exception);
}

TEST_F(EnergyEfficientAgentTest, adjust_platform_with_policy_freqs)
{
    const double min_freq = 1.0e9;
    const double max_freq = 2.0e9;
    const double mapped_region_1 = m_region_hashes[1];
    const double mapped_frequency_1 = 1.5e9;
    const double mapped_region_2 = m_region_hashes[3];
    const double mapped_frequency_2 = 1.6e9;
    std::vector<double> in_policy{ min_freq, max_freq, M_PERF_MARGIN,
                                   // Map a region
                                   mapped_region_1, mapped_frequency_1,
                                   // Unmapped region
                                   NAN, NAN,
                                   // Map another region. Doesn't need to be contiguous
                                   mapped_region_2, mapped_frequency_2 };

    EXPECT_CALL(*m_gov, set_frequency_bounds(min_freq, max_freq));

    for (const auto region : m_region_map) {
        // Frequencies are suggested ONLY for mapped regions
        if (region.first == mapped_region_1) {
            EXPECT_CALL(*region.second, suggest_freq(mapped_frequency_1)).Times(M_NUM_FREQ_DOMAIN);
        }
        else if (region.first == mapped_region_2) {
            EXPECT_CALL(*region.second, suggest_freq(mapped_frequency_2)).Times(M_NUM_FREQ_DOMAIN);
        }
        else {
            EXPECT_CALL(*region.second, suggest_freq(_)).Times(0);
        }
    }

    m_agent0->adjust_platform(in_policy);
}

TEST_F(EnergyEfficientAgentTest, sample_platform)
{
    ON_CALL(*m_gov, get_frequency_min()).WillByDefault(Return(1e9));
    ON_CALL(*m_gov, get_frequency_max()).WillByDefault(Return(2e9));
    ON_CALL(*m_gov, get_frequency_step()).WillByDefault(Return(1e8));

    const uint64_t test_region = 0x12;
    const double test_frequency = 2.1e9;

    for (int idx = 0; idx < M_NUM_FREQ_DOMAIN; ++idx) {
        EXPECT_CALL(m_platio, sample(HASH_SIG + idx)).WillOnce(Return(test_region));
        EXPECT_CALL(m_platio, sample(HINT_SIG + idx)).WillOnce(Return(GEOPM_REGION_HINT_UNKNOWN));
        EXPECT_CALL(m_platio, sample(RUNTIME_SIG + idx)).WillOnce(Return(0.1));
        EXPECT_CALL(m_platio, sample(COUNT_SIG + idx)).WillOnce(Return(1));
    }

    // First sample: initialize regions for the first time
    const size_t expected_size = m_agent0->sample_names().size();
    std::vector<double> out_sample(expected_size, NAN);
    m_agent0->sample_platform(out_sample);
    EXPECT_THAT(out_sample, IsEqualToPolicy({}, expected_size));
    EXPECT_FALSE(m_agent0->do_send_sample());

    // Prepare the second sample: all instances in the frequency domain have
    // re-entered the region
    for (int idx = 0; idx < M_NUM_FREQ_DOMAIN; ++idx) {
        EXPECT_CALL(m_platio, sample(HASH_SIG + idx)).WillOnce(Return(test_region));
        EXPECT_CALL(m_platio, sample(HINT_SIG + idx)).WillOnce(Return(GEOPM_REGION_HINT_UNKNOWN));
        EXPECT_CALL(m_platio, sample(RUNTIME_SIG + idx)).WillOnce(Return(0.1));
        EXPECT_CALL(m_platio, sample(COUNT_SIG + idx)).WillOnce(Return(2));
    }
    // Expect that learning has finished at the test frequency
    EXPECT_CALL(*m_region_map[test_region], update_exit(_))
        .Times(M_NUM_FREQ_DOMAIN)
        .WillRepeatedly(Return(true));
    EXPECT_CALL(*m_region_map[test_region], freq())
        .Times(M_NUM_FREQ_DOMAIN)
        .WillRepeatedly(Return(test_frequency));

    m_agent0->sample_platform(out_sample);

    EXPECT_THAT(out_sample,
                IsEqualToPolicy({ test_region, test_frequency }, expected_size));
    EXPECT_TRUE(m_agent0->do_send_sample());
}

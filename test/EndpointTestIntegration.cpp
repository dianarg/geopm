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

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Endpoint.hpp"
#include "EndpointUser.hpp"


// TODO: these can probably be combined into one test class

class EndpointTestIntegration : public ::testing::Test
{
    protected:
        void TearDown();
        const std::string m_shm_path = "/EndpointTestIntegration_data_" + std::to_string(geteuid());
};

class EndpointUserTestIntegration : public ::testing::Test
{
    protected:
        void TearDown();
        const std::string m_shm_path = "/EndpointUserTestIntegration_data_" + std::to_string(geteuid());
};


void EndpointTestIntegration::TearDown()
{
    unlink(("/dev/shm/" + m_shm_path + "-policy").c_str());
    unlink(("/dev/shm/" + m_shm_path + "-sample").c_str());
}

void EndpointUserTestIntegration::TearDown()
{
    unlink(("/dev/shm/" + m_shm_path + "-policy").c_str());
    unlink(("/dev/shm/" + m_shm_path + "-sample").c_str());
}

TEST_F(EndpointTestIntegration, write_shm)
{
    std::vector<double> values = {777, 12.3456, 2.1e9, 5.5};
    EndpointImp end(m_shm_path, nullptr, nullptr);
    end.open();
    SharedMemoryUserImp p_smp(m_shm_path + "-policy", 1);
    struct geopm_endpoint_policy_shmem_s *p_data = (struct geopm_endpoint_policy_shmem_s *) p_smp.pointer();
    SharedMemoryUserImp s_smp(m_shm_path + "-sample", 1);
    struct geopm_endpoint_sample_shmem_s *s_data = (struct geopm_endpoint_sample_shmem_s *) s_smp.pointer();
    std::string agent = "energy_efficient";
    strncpy(s_data->agent, agent.c_str(), agent.size());

    end.write_policy(values);

    ASSERT_LT(0u, p_data->count);
    std::vector<double> result(p_data->values, p_data->values + p_data->count);
    EXPECT_EQ(values, result);

    values[0] = 888;

    end.write_policy(values);
    ASSERT_LT(0u, p_data->count);
    std::vector<double> result2(p_data->values, p_data->values + p_data->count);
    EXPECT_EQ(values, result2);
    end.close();
}

TEST_F(EndpointTestIntegration, write_read_policy)
{
    std::vector<double> values = {777, 12.3456, 2.1e9, 5.5};
    EndpointImp end(m_shm_path, nullptr, nullptr);
    end.open();
    EndpointUserImp user(m_shm_path, nullptr, nullptr, "energy_efficient", 0, "", "", {});

    end.write_policy(values);

    std::vector<double> result(values.size());
    user.read_policy(result);
    EXPECT_EQ(values, result);

    values[0] = 888;
    end.write_policy(values);
    double age = user.read_policy(result);
    EXPECT_EQ(values, result);
    EXPECT_LT(0.0, age);
    EXPECT_LT(age, 0.01);

    // clear() wipes policy data
    end.clear();
    result = {};  // TODO: weird; should read_policy() be disabled if no agent attached?
    user.read_policy(result);
    values = {};
    EXPECT_EQ(values, result);

    end.close();
}

TEST_F(EndpointTestIntegration, write_read_sample)
{
    std::vector<double> values = {777, 12.3456, 2.1e9, 2.3e9};
    std::set<std::string> hosts = {"node5", "node6", "node8"};
    std::string hostlist_path = "EndpointTestIntegration_hostlist";
    EndpointImp end(m_shm_path, nullptr, nullptr);
    end.open();
    EndpointUserImp user(m_shm_path, nullptr, nullptr, "power_balancer",
                         values.size(), "myprofile", hostlist_path, hosts);
    EXPECT_EQ("power_balancer", end.get_agent());
    EXPECT_EQ("myprofile", end.get_profile_name());
    EXPECT_EQ(hosts, end.get_hostnames());

    user.write_sample(values);
    std::vector<double> result(values.size());
    end.read_sample(result);
    EXPECT_EQ(values, result);

    values[0] = 888;
    user.write_sample(values);
    end.read_sample(result);
    EXPECT_EQ(values, result);

    // clear() wipes sample data
    end.clear();
    result = {}; // TODO: weird; should read_sample be disabled if no agent attached?
    end.read_sample(result);
    values = {};
    EXPECT_EQ(values, result);

    end.close();
    unlink(hostlist_path.c_str());
}

TEST_F(EndpointUserTestIntegration, parse_shm)
{
    std::string full_path("/dev/shm" + m_shm_path);

    size_t shmem_size = sizeof(struct geopm_endpoint_policy_shmem_s);
    SharedMemoryImp smp(m_shm_path + "-policy", shmem_size);
    struct geopm_endpoint_policy_shmem_s *data = (struct geopm_endpoint_policy_shmem_s *) smp.pointer();
    SharedMemoryImp sms(m_shm_path + "-sample", sizeof(struct geopm_endpoint_sample_shmem_s));

    double tmp[] = { 1.1, 2.2, 3.3 };
    int num_policy = sizeof(tmp) / sizeof(tmp[0]);
    geopm_time_s now;
    geopm_time(&now);
    data->count = num_policy;
    // Build the data
    memcpy(data->values, tmp, sizeof(tmp));
    geopm_time(&data->timestamp);

    EndpointUserImp gp(m_shm_path, nullptr, nullptr, "myagent", 0, "myprofile", "", {});

    std::vector<double> result(num_policy);
    double age = gp.read_policy(result);
    EXPECT_LT(0.0, age);
    EXPECT_LT(age, 0.01);
    std::vector<double> expected {tmp, tmp + num_policy};
    EXPECT_EQ(expected, result);

    tmp[0] = 1.5;
    memcpy(data->values, tmp, sizeof(tmp));

    gp.read_policy(result);
    expected = {tmp, tmp + num_policy};
    EXPECT_EQ(expected, result);
}

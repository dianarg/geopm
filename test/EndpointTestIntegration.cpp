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

class EndpointTestIntegration : public ::testing::Test
{
    protected:
        void TearDown();
        const std::string m_shm_path = "/EndpointTestIntegration_data_" + std::to_string(geteuid());
};

void EndpointTestIntegration::TearDown()
{
    unlink(("/dev/shm/" + m_shm_path + "-policy").c_str());
    unlink(("/dev/shm/" + m_shm_path + "-sample").c_str());
}

TEST_F(EndpointTestIntegration, write_read_policy)
{
    std::vector<double> values = {777, 12.3456, 2.1e9, 5.5};
    //EndpointImp end(m_shm_path, nullptr, nullptr);
    auto end = Endpoint::make_unique(m_shm_path);
    end.open();
    auto user = EndpointUser::make_unique(m_shm_path, "energy_efficient", {});
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
    //    EndpointUserImp user(m_shm_path, nullptr, nullptr, "power_balancer",
    //                     values.size(), "myprofile", hostlist_path, hosts);
    auto user = EndpointUser::make_unique(m_shm_path, "power_balancer", "myprofile", hosts);
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

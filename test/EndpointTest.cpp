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

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <future>
#include <thread>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "geopm_test.hpp"

#include "geopm_time.h"
#include "geopm_endpoint.h"
#include "MockSharedMemory.hpp"
#include "MockAgent.hpp"
#include "Helper.hpp"
#include "Exception.hpp"
#include "EndpointImp.hpp"
#include "EndpointUser.hpp"
#include "SharedMemoryImp.hpp"

using geopm::EndpointImp;
using geopm::EndpointUserImp;
using geopm::SharedMemoryUserImp;
using geopm::geopm_endpoint_policy_shmem_s;
using geopm::geopm_endpoint_sample_shmem_s;
using geopm::Exception;
using testing::AtLeast;

class EndpointTest : public ::testing::Test
{
    protected:
        static void SetUpTestCase();
        void SetUp();
        const std::string m_shm_path = "/EndpointTest_data_" + std::to_string(geteuid());
        std::unique_ptr<MockSharedMemory> m_policy_shmem;
        std::unique_ptr<MockSharedMemory> m_sample_shmem;
        // Injection points into mock shared memory owned by endpoint
        struct geopm_endpoint_policy_shmem_s *m_policy_data_ptr;
        struct geopm_endpoint_sample_shmem_s *m_sample_data_ptr;

        int m_timeout;
        static std::string m_agent;
};


std::unique_ptr<geopm::Agent> make_mock_agent(void)
{
    return geopm::make_unique<MockAgent>();
}

std::string EndpointTest::m_agent = "mock";

void EndpointTest::SetUpTestCase()
{
    // add mock agent to factory with 3 policy values and 5 sample values
    geopm::agent_factory().register_plugin(m_agent, make_mock_agent,
                                           geopm::Agent::make_dictionary({"p1", "p2", "p3"},
                                                                         {"s1", "s2", "s3", "s4", "s5"}));
}

/// Used to mock other side of endpoint by explicitly writing to shmem
class EndpointTestMockUser
{
    public:
        EndpointTestMockUser(struct geopm_endpoint_policy_shmem_s *p_data,
                             struct geopm_endpoint_sample_shmem_s *s_data);
        virtual ~EndpointTestMockUser();
        void set_agent(const std::string &agent);
        void set_profile(const std::string &profile);
        void set_hostlist(const std::set<std::string> &hosts);
        std::vector<double> get_policy(void);
        void set_sample_values(const std::vector<double> &values);
        std::string hostlist_path(void) const;
    private:
        struct geopm_endpoint_policy_shmem_s *m_policy_ptr;
        struct geopm_endpoint_sample_shmem_s *m_sample_ptr;
        std::string m_hostlist_path;
};

EndpointTestMockUser::EndpointTestMockUser(struct geopm_endpoint_policy_shmem_s *p_data,
                                           struct geopm_endpoint_sample_shmem_s *s_data)
    : m_policy_ptr(p_data)
    , m_sample_ptr(s_data)
    , m_hostlist_path("EndpointTest_hostlist")
{

}

EndpointTestMockUser::~EndpointTestMockUser()
{
    unlink(m_hostlist_path.c_str());
}

void EndpointTestMockUser::set_agent(const std::string &agent)
{
    strncpy(m_sample_ptr->agent, agent.c_str(), agent.size());
    m_sample_ptr->agent[agent.size()] = '\0';
}

void EndpointTestMockUser::set_profile(const std::string &profile)
{
    strncpy(m_sample_ptr->profile_name, profile.c_str(), profile.size());
    m_sample_ptr->profile_name[profile.size()] = '\0';
}

void EndpointTestMockUser::set_hostlist(const std::set<std::string> &hosts)
{
    std::ofstream hostlist(m_hostlist_path);
    for (auto host : hosts) {
        hostlist << host << "\n";
    }
    hostlist.close();
    strncpy(m_sample_ptr->hostlist_path, m_hostlist_path.c_str(), m_hostlist_path.size());
    m_sample_ptr->hostlist_path[m_hostlist_path.size()] = '\0';
}

std::vector<double> EndpointTestMockUser::get_policy(void)
{
    std::vector<double> result(m_policy_ptr->values, m_policy_ptr->values + m_policy_ptr->count);
    return result;
}

// could be generic function used for both sample and policy?  pointer type is different
void EndpointTestMockUser::set_sample_values(const std::vector<double> &values)
{
    // todo: should be 3 separate functions?  this is very close to actual implementation
    // separate functions might allow testing some misbehavior?
    // i.e. there is a shmem lock, but no concept of transaction, so we can still have
    // a partial write
    m_sample_ptr->count = values.size();
    memcpy(m_sample_ptr->values, values.data(), values.size());
    geopm_time_s now;
    geopm_time(&now);
    m_sample_ptr->timestamp = now;
}

std::string EndpointTestMockUser::hostlist_path(void) const
{
    return m_hostlist_path;
}

void EndpointTest::SetUp()
{
    size_t policy_shmem_size = sizeof(struct geopm_endpoint_policy_shmem_s);
    m_policy_shmem = geopm::make_unique<MockSharedMemory>(policy_shmem_size);
    size_t sample_shmem_size = sizeof(struct geopm_endpoint_sample_shmem_s);
    m_sample_shmem = geopm::make_unique<MockSharedMemory>(sample_shmem_size);
    m_policy_data_ptr = (struct geopm_endpoint_policy_shmem_s *) m_policy_shmem->pointer();
    m_sample_data_ptr = (struct geopm_endpoint_sample_shmem_s *) m_sample_shmem->pointer();

    m_timeout = 2;

    EXPECT_CALL(*m_policy_shmem, get_scoped_lock()).Times(AtLeast(0));
    EXPECT_CALL(*m_policy_shmem, unlink());
    EXPECT_CALL(*m_sample_shmem, get_scoped_lock()).Times(AtLeast(0));
    EXPECT_CALL(*m_sample_shmem, unlink());
}


TEST_F(EndpointTest, write_shm_policy)
{
    std::vector<double> values = {777, 12.3456, 2.3e9};
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    user.set_agent(m_agent);
    end.write_policy(values);
    std::vector<double> test = user.get_policy();
    EXPECT_EQ(values, test);
    end.close();
}

TEST_F(EndpointTest, parse_shm_sample)
{
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    // Build the data
    user.set_agent(m_agent);
    std::vector<double> values { 1.1, 2.2, 3.3, 4.4, 5.5 };
    user.set_sample_values(values);
    std::vector<double> result(values.size());
    double age = end.read_sample(result);
    EXPECT_EQ(values, result);
    EXPECT_LT(0.0, age);
    EXPECT_LT(age, 0.01);
    end.close();
}

TEST_F(EndpointTest, get_agent)
{
    struct geopm_endpoint_sample_shmem_s *data = (struct geopm_endpoint_sample_shmem_s *) m_sample_shmem->pointer();
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    end.open();
    strncpy(data->agent, "monitor", GEOPM_ENDPOINT_AGENT_NAME_MAX);
    EXPECT_EQ("monitor", end.get_agent());
    end.clear();
    EXPECT_EQ("", end.get_agent());
    end.close();
}

TEST_F(EndpointTest, get_profile_name)
{
    struct geopm_endpoint_sample_shmem_s *data = (struct geopm_endpoint_sample_shmem_s *) m_sample_shmem->pointer();
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    user.set_profile("my_prof");
    EXPECT_EQ("my_prof", end.get_profile_name());
    end.clear();
    EXPECT_EQ("", end.get_profile_name());
    end.close();
}

TEST_F(EndpointTest, get_hostnames)
{
    std::set<std::string> hosts = {"node0", "node1", "node2", "node3", "node4"};
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    user.set_hostlist(hosts);
    user.set_agent(m_agent);
    EXPECT_EQ(hosts, end.get_hostnames());
    end.clear();
    EXPECT_EQ(std::set<std::string>(), end.get_hostnames());
    // assert that even the hostnames file itself has been cleaned up
    std::ifstream test_open(user.hostlist_path());
    EXPECT_FALSE(test_open.good());
    end.close();
}

TEST_F(EndpointTest, stop_wait_loop)
{
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    end.open();
    end.reset_wait_loop();
    auto run_thread = std::async(std::launch::async,
                                 [&end, this] {
                                     end.wait_for_agent_attach(m_timeout);
                                 });
    end.stop_wait_loop();
    // wait for less than timeout; should exit before time limit without throwing
    auto result = run_thread.wait_for(std::chrono::seconds(m_timeout - 1));
    EXPECT_NE(result, std::future_status::timeout);
    end.close();
}

TEST_F(EndpointTest, attach_wait_loop_timeout_throws)
{
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    end.open();
    geopm_time_s before;
    geopm_time(&before);
    auto run_thread = std::async(std::launch::async,
                                 [&end, this] {
                                     end.wait_for_agent_attach(m_timeout);
                                 });
    // throw from our timeout should happen before longer async timeout
    std::future_status result = run_thread.wait_for(std::chrono::seconds(m_timeout + 1));
    GEOPM_EXPECT_THROW_MESSAGE(run_thread.get(),
                               GEOPM_ERROR_RUNTIME,
                               "timed out");
    EXPECT_NE(result, std::future_status::timeout);
    double elapsed = geopm_time_since(&before);
    EXPECT_NEAR(m_timeout, elapsed, 0.100);
    end.close();
}

TEST_F(EndpointTest, detach_wait_loop_timeout_throws)
{
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    // simulate agent attach
    user.set_agent("monitor");

    geopm_time_s before;
    geopm_time(&before);
    auto run_thread = std::async(std::launch::async,
                                 [&end, this] {
                                     end.wait_for_agent_detach(m_timeout);
                                 });
    // throw from our timeout should happen before longer async timeout
    std::future_status result = run_thread.wait_for(std::chrono::seconds(m_timeout + 1));
    GEOPM_EXPECT_THROW_MESSAGE(run_thread.get(),
                               GEOPM_ERROR_RUNTIME,
                               "timed out");
    EXPECT_NE(result, std::future_status::timeout);
    double elapsed = geopm_time_since(&before);
    EXPECT_NEAR(m_timeout, elapsed, 0.100);
    end.close();
}

TEST_F(EndpointTest, wait_stops_when_agent_attaches)
{
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    auto run_thread = std::async(std::launch::async,
                                 [&end, this] {
                                     end.wait_for_agent_attach(m_timeout);
                                 });
    // simulate agent attach
    user.set_agent("monitor");
    // wait for less than timeout; should exit before time limit without throwing
    auto result = run_thread.wait_for(std::chrono::seconds(m_timeout - 1));
    EXPECT_NE(result, std::future_status::timeout);
    EXPECT_EQ("monitor", end.get_agent());
    end.close();
}

TEST_F(EndpointTest, wait_attach_timeout_0)
{
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    // if agent is not already attached, throw immediately
    GEOPM_EXPECT_THROW_MESSAGE(end.wait_for_agent_attach(0), GEOPM_ERROR_RUNTIME, "timed out");
    // simulate agent attach
    user.set_agent("monitor");
    // once agent is attached, timeout of 0 should succeed
    end.wait_for_agent_attach(0);
    EXPECT_EQ("monitor", end.get_agent());
    end.close();
}

TEST_F(EndpointTest, wait_stops_when_agent_detaches)
{
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    // simulate agent attach
    user.set_agent("monitor");
    ASSERT_EQ("monitor", end.get_agent());
    auto run_thread = std::async(std::launch::async,
                                 [&end, this] {
                                     end.wait_for_agent_detach(m_timeout);
                                 });
    // simulate agent detach
    user.set_agent("");
    // wait for less than timeout; should exit before time limit without throwing
    auto result = run_thread.wait_for(std::chrono::seconds(m_timeout - 1));
    EXPECT_NE(result, std::future_status::timeout);
    EXPECT_EQ("", end.get_agent());
    end.close();
}

TEST_F(EndpointTest, wait_detach_timeout_0)
{
    EndpointImp end(m_shm_path, std::move(m_policy_shmem), std::move(m_sample_shmem));
    EndpointTestMockUser user(m_policy_data_ptr, m_sample_data_ptr);
    end.open();
    // simulate agent attach
    user.set_agent("monitor");
    // if agent is still attached, throw immediately for timeout of 0
    GEOPM_EXPECT_THROW_MESSAGE(end.wait_for_agent_detach(0), GEOPM_ERROR_RUNTIME, "timed out");

    // simulate agent detach
    user.set_agent("");
    // once agent is detached, timeout of 0 should succeed
    end.wait_for_agent_detach(0);
    EXPECT_EQ("", end.get_agent());
    end.close();
}

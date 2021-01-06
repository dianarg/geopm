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

#include <sched.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <future>
#include <chrono>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "geopm.h"
#include "Profile.hpp"
#include "SharedMemory.hpp"
#include "ApplicationRecordLog.hpp"
#include "ApplicationStatus.hpp"

#include "geopm_test.hpp"
#include "MockComm.hpp"

#include "MockControlMessage.hpp"
#include "MockProfileTable.hpp"


using geopm::Profile;
using geopm::ProfileImp;
using geopm::SharedMemory;
using geopm::ApplicationRecordLog;
using geopm::record_s;
using geopm::short_region_s;
using geopm::ApplicationStatus;
using testing::Return;
using testing::NiceMock;
using testing::_;


class ProfileTestPerformance : public ::testing::Test
{
    protected:
        void SetUp();
        void TearDown();

        const int M_NUM_CPU = 4;
        const int m_process = 42;
        std::set<int> m_cpu_list = {2, 3};
        const int m_timeout = 1;
        std::string M_SHM_KEY = "ProfileTestPerformance";

        // legacy code path
        std::shared_ptr<MockComm> m_world_comm;
        std::shared_ptr<MockComm> m_shm_comm;
        std::shared_ptr<MockComm> m_comm;
        std::shared_ptr<SharedMemory> m_table_shm;
        std::shared_ptr<MockProfileTable> m_table;
        std::shared_ptr<MockControlMessage> m_ctl_msg;

        // new code path
        std::shared_ptr<SharedMemory> m_ctl_record_shmem;
        std::shared_ptr<SharedMemory> m_ctl_status_shmem;
        std::shared_ptr<ApplicationRecordLog> m_ctl_record_log;
        std::shared_ptr<ApplicationStatus> m_ctl_status;
        std::shared_ptr<Profile> m_profile;
};

void ProfileTestPerformance::SetUp()
{
    int shm_rank = 6;
    std::string M_PROF_NAME = "profile_test";
    std::string M_REPORT = "report_test";
    int M_SHM_COMM_SIZE = 2;

    // new code path
    m_ctl_record_shmem = SharedMemory::make_unique_owner(M_SHM_KEY + "-record-log-42",
                                                         ApplicationRecordLog::buffer_size());
    m_ctl_status_shmem = SharedMemory::make_unique_owner(M_SHM_KEY + "-status",
                                                         ApplicationStatus::buffer_size(M_NUM_CPU));
    m_ctl_record_log = ApplicationRecordLog::make_unique(m_ctl_record_shmem);
    m_ctl_status = ApplicationStatus::make_unique(M_NUM_CPU, m_ctl_status_shmem);

    // legacy
    m_ctl_msg = std::make_shared<NiceMock<MockControlMessage> >();
    m_shm_comm = std::make_shared<NiceMock<MockComm> >();
    ON_CALL(*m_shm_comm, rank()).WillByDefault(Return(shm_rank));
    ON_CALL(*m_shm_comm, num_rank()).WillByDefault(Return(M_SHM_COMM_SIZE));
    ON_CALL(*m_shm_comm, test(testing::_))
        .WillByDefault(testing::Return(true));

    m_world_comm = std::make_shared<NiceMock<MockComm> >();
    ON_CALL(*m_world_comm, rank()).WillByDefault(Return(m_process));
    ON_CALL(*m_world_comm, split("prof", geopm::Comm::M_COMM_SPLIT_TYPE_SHARED))
        .WillByDefault(Return(m_shm_comm));
    m_comm = std::make_shared<NiceMock<MockComm> >();
    m_table = std::make_shared<MockProfileTable>();
    ON_CALL(*m_table, name_fill(_))
        .WillByDefault(Return(true));

    m_profile = geopm::make_unique<ProfileImp>(M_PROF_NAME,
                                               M_SHM_KEY,
                                               M_REPORT,
                                               m_timeout,
                                               m_world_comm,
                                               m_ctl_msg,
                                               M_NUM_CPU,
                                               m_cpu_list,
                                               m_table,
                                               m_comm,
                                               nullptr,  // status
                                               nullptr); // record_log

    m_profile->init();
}

void ProfileTestPerformance::TearDown()
{
    // Note: owner side sharedmemory unlink won't throw
    if (m_table_shm) {
        m_table_shm->unlink();
    }
    if (m_ctl_record_shmem) {
        m_ctl_record_shmem->unlink();
    }
    if (m_ctl_status_shmem) {
        m_ctl_status_shmem->unlink();
    }
}

TEST_F(ProfileTestPerformance, enter_exit)
{
    uint64_t hash = 0xABCD;
    uint64_t hint = GEOPM_REGION_HINT_COMPUTE;
    uint64_t region_id = hint | hash;

    // Controller thread
    volatile bool run_controller = true;
    std::future<void> ctl = std::async(
        std::launch::async, [&run_controller, this]
        {
            std::vector<record_s> records;
            std::vector<short_region_s> short_regions;
            double WAIT_SEC = 0.005;
            geopm_time_s last_wait;
            geopm_time(&last_wait);
            while (run_controller) {
                while(geopm_time_since(&last_wait) < WAIT_SEC) {
                }
                geopm_time(&last_wait);

                m_ctl_record_log->dump(records, short_regions);
                m_ctl_status->update_cache();
            }
        });

    geopm_time_s start_time, end_time;
    int num_iter = 1000;
    geopm_time(&start_time);
    for (int ii = 0; ii < num_iter; ++ii) {
        m_profile->enter(0xA + ii);
        m_profile->exit(0xA + ii);
    }
    geopm_time(&end_time);
    run_controller = false;

    ctl.wait_for(std::chrono::seconds(1));
    ctl.get();

}

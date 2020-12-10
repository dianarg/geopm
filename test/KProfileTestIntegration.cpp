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

#include "Profile.hpp"

#include <sched.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <functional>
#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "geopm.h"
#include "Profile.hpp"
#include "SharedMemory.hpp"
#include "ApplicationRecordLog.hpp"
#include "ApplicationStatus.hpp"

#include "geopm_test.hpp"
#include "MockPlatformTopo.hpp"
#include "MockComm.hpp"

#include "MockControlMessage.hpp"
#include "MockPlatformTopo.hpp"
#include "MockProfileTable.hpp"
#include "MockProfileThreadTable.hpp"
#include "MockSampleScheduler.hpp"


using geopm::Profile;
using geopm::ProfileImp;
using geopm::KProfileImp;
using geopm::SharedMemory;
using geopm::ApplicationRecordLog;
using geopm::record_s;
using geopm::short_region_s;
using geopm::ApplicationStatus;
using testing::Return;
using testing::NiceMock;

class ProfileTestProfileTable;
class ProfileTestProfileThreadTable;
class ProfileTestSampleScheduler;
class ProfileTestControlMessage;

class KProfileTestIntegration : public ::testing::Test
{
    protected:
        void SetUp();
        void TearDown();

        const int M_NUM_CPU = 4;
        MockPlatformTopo m_topo;
        const int m_process = 42;
        const int m_timeout = 1;
        std::string M_SHM_KEY = "KProfileTestIntegration";

        // legacy code path
        std::unique_ptr<MockComm> m_world_comm;
        std::shared_ptr<MockComm> m_shm_comm;
        std::shared_ptr<MockComm> m_comm;
        std::shared_ptr<SharedMemory> m_table_shm;
        std::unique_ptr<ProfileTestProfileTable> m_table;
        std::shared_ptr<SharedMemory> m_tprof_shm;
        std::unique_ptr<ProfileTestProfileThreadTable> m_tprof;
        std::unique_ptr<ProfileTestSampleScheduler> m_scheduler;
        std::unique_ptr<ProfileTestControlMessage> m_ctl_msg;


        // new code path
        std::shared_ptr<SharedMemory> m_ctl_record_shmem;
        std::shared_ptr<SharedMemory> m_ctl_status_shmem;
        std::shared_ptr<ApplicationRecordLog> m_ctl_record_log;
        std::shared_ptr<ApplicationStatus> m_ctl_status;
        std::shared_ptr<Profile> m_profile;
        std::shared_ptr<KProfileImp> m_kprofile;
};

void KProfileTestIntegration::SetUp()
{
    ON_CALL(m_topo, num_domain(GEOPM_DOMAIN_CPU))
        .WillByDefault(Return(M_NUM_CPU));
    EXPECT_CALL(m_topo, num_domain(testing::_)).Times(testing::AtLeast(0));

    int shm_rank = 6;
    int world_rank = 37;
    std::string M_PROF_NAME = "profile_test";
    std::string M_REPORT = "report_test";
    bool M_DO_REGION_BARRIER = false;
    int M_SHMEM_REGION_SIZE = 12288;
    int M_SHM_COMM_SIZE = 2;

    // new code path
    m_ctl_record_shmem = SharedMemory::make_unique_owner(M_SHM_KEY + "-record-log-42",
                                                         ApplicationRecordLog::buffer_size());
    m_ctl_status_shmem = SharedMemory::make_unique_owner(M_SHM_KEY + "-status",
                                                         ApplicationStatus::buffer_size(M_NUM_CPU));
    m_ctl_record_log = ApplicationRecordLog::make_unique(m_ctl_record_shmem);
    m_ctl_status = ApplicationStatus::make_unique(m_topo, m_ctl_status_shmem);

    // init() will connect to shared memory inside of Profile::init()
    m_kprofile = geopm::make_unique<KProfileImp>(m_topo,
                                                M_SHM_KEY,
                                                m_timeout,
                                                nullptr,
                                                nullptr,
                                                m_process);

    // legacy
    m_ctl_msg = geopm::make_unique<ProfileTestControlMessage>();
    m_shm_comm = std::make_shared<NiceMock<MockComm> >();
    ON_CALL(*m_shm_comm, rank()).WillByDefault(Return(shm_rank));
    ON_CALL(*m_shm_comm, num_rank()).WillByDefault(Return(M_SHM_COMM_SIZE));
    ON_CALL(*m_shm_comm, test(testing::_))
        .WillByDefault(testing::Return(true));

    m_world_comm = geopm::make_unique<NiceMock<MockComm> >();
    ON_CALL(*m_world_comm, rank()).WillByDefault(Return(world_rank));
    ON_CALL(*m_world_comm, split("prof", geopm::Comm::M_COMM_SPLIT_TYPE_SHARED))
        .WillByDefault(Return(m_shm_comm));
    m_comm = std::make_shared<NiceMock<MockComm> >();

    m_scheduler = geopm::make_unique<ProfileTestSampleScheduler>();
    m_tprof_shm = SharedMemory::make_unique_owner(M_SHM_KEY + "-tprof", M_NUM_CPU * 64);
    std::string table_shm_key = M_SHM_KEY + "-sample-" + std::to_string(world_rank);
    m_table_shm = SharedMemory::make_unique_owner(table_shm_key, M_SHMEM_REGION_SIZE);

    m_profile = geopm::make_unique<ProfileImp>(M_PROF_NAME, M_SHM_KEY, M_REPORT,
                                               m_timeout, M_DO_REGION_BARRIER,
                                               std::move(m_world_comm),
                                               std::move(m_ctl_msg), m_topo,
                                               nullptr, nullptr, std::move(m_scheduler),
                                               m_comm, m_kprofile);

    m_profile->init();
}

void KProfileTestIntegration::TearDown()
{
    // Note: owner side sharedmemory unlink won't throw
    if (m_table_shm) {
        m_table_shm->unlink();
    }
    if (m_tprof_shm) {
        m_tprof_shm->unlink();
    }
    if (m_ctl_record_shmem) {
        m_ctl_record_shmem->unlink();
    }
    if (m_ctl_status_shmem) {
        m_ctl_status_shmem->unlink();
    }
}

TEST_F(KProfileTestIntegration, enter_exit)
{
    uint64_t hash = 0xABCD;
    uint64_t hint = GEOPM_REGION_HINT_COMPUTE;
    uint64_t region_id = hint | hash;
    std::vector<record_s> records;
    std::vector<short_region_s> short_regions;

    m_profile->enter(region_id);
    m_ctl_record_log->dump(records, short_regions);
    ASSERT_EQ(1ull, records.size());
    EXPECT_EQ(m_process, records[0].process);
    EXPECT_EQ(geopm::EVENT_REGION_ENTRY, records[0].event);
    EXPECT_EQ(hash, records[0].signal);
    EXPECT_EQ(hint, m_ctl_status->get_hint(0));

    m_profile->exit(region_id);
    m_ctl_record_log->dump(records, short_regions);
    ASSERT_EQ(1ull, records.size());
    EXPECT_EQ(m_process, records[0].process);
    EXPECT_EQ(geopm::EVENT_REGION_EXIT, records[0].event);
    EXPECT_EQ(hash, records[0].signal);
    EXPECT_EQ(0ULL, m_ctl_status->get_hint(0));

}

TEST_F(KProfileTestIntegration, enter_exit_short)
{
    uint64_t hash = 0xABCD;
    uint64_t hint = GEOPM_REGION_HINT_COMPUTE;
    uint64_t region_id = hint | hash;
    std::vector<record_s> records;
    std::vector<short_region_s> short_regions;

    m_profile->enter(region_id);
    m_profile->exit(region_id);
    m_ctl_record_log->dump(records, short_regions);
    ASSERT_EQ(1ULL, records.size());
    EXPECT_EQ(m_process, records[0].process);
    EXPECT_EQ(geopm::EVENT_SHORT_REGION, records[0].event);
    EXPECT_EQ(0ULL, records[0].signal);
    ASSERT_EQ(1ULL, short_regions.size());
    EXPECT_EQ(hash, short_regions[0].hash);
    EXPECT_EQ(1, short_regions[0].num_complete);
    // exited region
    EXPECT_EQ(0ULL, m_ctl_status->get_hint(0));


    m_profile->enter(region_id);
    m_profile->exit(region_id);
    m_profile->enter(region_id);
    m_profile->exit(region_id);
    m_profile->enter(region_id);
    m_ctl_record_log->dump(records, short_regions);
    ASSERT_EQ(1ULL, records.size());
    EXPECT_EQ(m_process, records[0].process);
    EXPECT_EQ(geopm::EVENT_SHORT_REGION, records[0].event);
    EXPECT_EQ(0ULL, records[0].signal);
    ASSERT_EQ(1ULL, short_regions.size());
    EXPECT_EQ(hash, short_regions[0].hash);
    EXPECT_EQ(2, short_regions[0].num_complete);
    // still in region
    EXPECT_EQ(hint, m_ctl_status->get_hint(0));

    m_profile->exit(region_id);
    m_ctl_record_log->dump(records, short_regions);
    ASSERT_EQ(1ULL, records.size());
    EXPECT_EQ(m_process, records[0].process);
    EXPECT_EQ(geopm::EVENT_SHORT_REGION, records[0].event);
    EXPECT_EQ(0ULL, records[0].signal);
    ASSERT_EQ(1ULL, short_regions.size());
    EXPECT_EQ(hash, short_regions[0].hash);
    EXPECT_EQ(1, short_regions[0].num_complete);
    // exited region
    EXPECT_EQ(0ULL, m_ctl_status->get_hint(0));

}

TEST_F(KProfileTestIntegration, enter_exit_nested)
{
    uint64_t usr_hash = 0xABCD;
    uint64_t usr_hint = GEOPM_REGION_HINT_COMPUTE;
    uint64_t usr_region_id = usr_hint | usr_hash;
    uint64_t mpi_hash = 0x5678;
    uint64_t mpi_hint = GEOPM_REGION_HINT_NETWORK;
    uint64_t mpi_region_id = mpi_hint | mpi_hash;
    std::vector<record_s> records;
    std::vector<short_region_s> short_regions;

    m_profile->enter(usr_hint | usr_hash);
    m_ctl_record_log->dump(records, short_regions);
    ASSERT_EQ(1ULL, records.size());
    EXPECT_EQ(geopm::EVENT_REGION_ENTRY, records[0].event);
    EXPECT_EQ(usr_hash, records[0].signal);
    EXPECT_EQ(usr_hint, m_ctl_status->get_hint(0));

    m_profile->enter(mpi_hint | mpi_hash);
    m_ctl_record_log->dump(records, short_regions);
    // no entry for nested region
    ASSERT_EQ(0ULL, records.size());
    EXPECT_EQ(mpi_hint, m_ctl_status->get_hint(0));

    m_profile->exit(mpi_region_id);
    m_ctl_record_log->dump(records, short_regions);
    // no exit for nested region
    ASSERT_EQ(0ULL, records.size());
    EXPECT_EQ(usr_hint, m_ctl_status->get_hint(0));

    m_profile->exit(usr_region_id);
    m_ctl_record_log->dump(records, short_regions);
    ASSERT_EQ(1ULL, records.size());
    EXPECT_EQ(geopm::EVENT_REGION_EXIT, records[0].event);
    EXPECT_EQ(usr_hash, records[0].signal);
    EXPECT_EQ(0ULL, m_ctl_status->get_hint(0));
}

TEST_F(KProfileTestIntegration, epoch)
{
    std::vector<record_s> records;
    std::vector<short_region_s> short_regions;

    m_profile->epoch();

    m_ctl_record_log->dump(records, short_regions);
    ASSERT_EQ(1ULL, records.size());
    EXPECT_EQ(m_process, records[0].process);
    EXPECT_EQ(geopm::EVENT_EPOCH_COUNT, records[0].event);
    EXPECT_EQ(1ULL, records[0].signal);
    ASSERT_EQ(0ULL, short_regions.size());
}

TEST_F(KProfileTestIntegration, progress_multithread)
{
    m_profile->thread_init(2, 4);
    m_profile->thread_init(3, 8);

    m_profile->thread_post(3);
    m_profile->thread_post(2);
    EXPECT_EQ(0.25, m_ctl_status->get_progress_cpu(2));
    EXPECT_EQ(0.125, m_ctl_status->get_progress_cpu(3));

    m_profile->thread_post(3);
    EXPECT_EQ(0.25, m_ctl_status->get_progress_cpu(2));
    EXPECT_EQ(0.25, m_ctl_status->get_progress_cpu(3));

    m_profile->thread_post(2);
    m_profile->thread_post(2);
    m_profile->thread_post(3);
    EXPECT_EQ(0.75, m_ctl_status->get_progress_cpu(2));
    EXPECT_EQ(0.375, m_ctl_status->get_progress_cpu(3));

    m_profile->thread_post(3);
    m_profile->thread_post(2);
    EXPECT_EQ(1.0, m_ctl_status->get_progress_cpu(2));
    EXPECT_EQ(0.5, m_ctl_status->get_progress_cpu(3));
}



// Legacy mocks to be removed later
// copied from ProfileTest
class ProfileTestControlMessage : public MockControlMessage
{
    public:
        ProfileTestControlMessage()
        {
            EXPECT_CALL(*this, step())
                .WillRepeatedly(testing::Return());
            EXPECT_CALL(*this, wait())
                .WillRepeatedly(testing::Return());
            EXPECT_CALL(*this, cpu_rank(testing::_, testing::_))
                .WillRepeatedly(testing::Return());
            EXPECT_CALL(*this, cpu_rank(testing::_))
                .WillRepeatedly(testing::Return(0));
            EXPECT_CALL(*this, loop_begin())
                .WillRepeatedly(testing::Return());
        }
};

class ProfileTestSampleScheduler : public MockSampleScheduler
{
    public:
        ProfileTestSampleScheduler()
        {
            EXPECT_CALL(*this, clear())
                .WillRepeatedly(testing::Return());
            EXPECT_CALL(*this, do_sample())
                .WillRepeatedly(testing::Return(true));
        }
};

class ProfileTestPlatformTopo : public MockPlatformTopo
{
    public:
        ProfileTestPlatformTopo(int num_cpu)
        {
            EXPECT_CALL(*this, num_domain(GEOPM_DOMAIN_CPU))
                .WillRepeatedly(testing::Return(num_cpu));
        }
};

class ProfileTestProfileTable : public MockProfileTable
{
    public:
        ProfileTestProfileTable(std::function<uint64_t (const std::string &)> key_lambda,
                std::function<void (const struct geopm_prof_message_s &value)> insert_lambda)
        {
            EXPECT_CALL(*this, key(testing::_))
                .WillRepeatedly(testing::Invoke(key_lambda));
            EXPECT_CALL(*this, insert(testing::_))
                .WillRepeatedly(testing::Invoke(insert_lambda));
            EXPECT_CALL(*this, name_fill(testing::_))
                .WillRepeatedly(testing::Return(true));
        }
};

class ProfileTestProfileThreadTable : public MockProfileThreadTable
{
    public:
        ProfileTestProfileThreadTable(int num_cpu)
        {
            EXPECT_CALL(*this, num_cpu())
                .WillRepeatedly(testing::Return(num_cpu));
        }
};

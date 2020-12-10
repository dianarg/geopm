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

#include "geopm_test.hpp"
#include "MockPlatformTopo.hpp"
#include "MockApplicationRecordLog.hpp"
#include "MockApplicationStatus.hpp"

using geopm::Profile;
using geopm::KProfileImp;
using testing::_;
using testing::Return;

class KProfileTest : public ::testing::Test
{
    protected:
        void SetUp();
        const int m_proc_id = 42;
        std::list<int> m_cpu_list = {2, 3};
        MockPlatformTopo m_topo;
        std::shared_ptr<MockApplicationRecordLog> m_record_log;
        std::shared_ptr<MockApplicationStatus> m_status;
        // TODO: make this a base class ptr when interface is fixed
        std::unique_ptr<KProfileImp> m_profile;
};

void KProfileTest::SetUp()
{
    m_record_log = std::make_shared<MockApplicationRecordLog>();
    m_status = std::make_shared<MockApplicationStatus>();

    // TODO: replace with Profile factory method?

    EXPECT_CALL(*m_record_log, set_process(m_proc_id));
    EXPECT_CALL(*m_record_log, set_time_zero(_));
    m_profile = geopm::make_unique<KProfileImp>(m_topo, m_cpu_list, "shmem_key", 1,
                                                m_record_log, m_status, m_proc_id);
    m_profile->init();
}

TEST_F(KProfileTest, enter_exit)
{
    uint64_t hash = 0xABCD;
    uint64_t hint = GEOPM_REGION_HINT_COMPUTE;
    uint64_t region_id = hint | hash;
    EXPECT_CALL(*m_record_log, enter(hash, _));
    // TODO: get cpu ids for process; set hint on all
    EXPECT_CALL(*m_status, set_hint(0, hint));
    m_profile->enter(region_id);

    EXPECT_CALL(*m_record_log, exit(hash, _));
    // hint is cleared when exiting top-level region
    EXPECT_CALL(*m_status, set_hint(0, 0ULL));

    m_profile->exit(region_id);
}

// TODO: get rid of GEOPM_REGION_ID_MPI, epoch bit if still there
// TODO: fix geopm_mpi_region_enter/exit to set hint instead and
//       get rid of extra entry into GEOPM_REGION_ID_MPI

TEST_F(KProfileTest, enter_exit_nested)
{
    uint64_t usr_hash = 0xABCD;
    uint64_t usr_hint = GEOPM_REGION_HINT_COMPUTE;
    uint64_t usr_region_id = usr_hint | usr_hash;
    uint64_t mpi_hash = 0x5678;
    uint64_t mpi_hint = GEOPM_REGION_HINT_NETWORK;
    uint64_t mpi_region_id = mpi_hint | mpi_hash;
    {
        EXPECT_CALL(*m_record_log, enter(usr_hash, _));
        EXPECT_CALL(*m_status, set_hint(_, usr_hint));
        m_profile->enter(usr_hint | usr_hash);
    }
    {
        // don't enter a nested region, just update hint
        EXPECT_CALL(*m_record_log, enter(_, _)).Times(0);
        EXPECT_CALL(*m_status, set_hint(_, mpi_hint));
        m_profile->enter(mpi_hint | mpi_hash);
    }
    {
        // don't exit, just restore hint
        EXPECT_CALL(*m_record_log, exit(_, _)).Times(0);
        EXPECT_CALL(*m_status, set_hint(_, usr_hint));
        m_profile->exit(mpi_region_id);
    }
    {
        EXPECT_CALL(*m_record_log, exit(usr_hash, _));
        // unset hints
        EXPECT_CALL(*m_status, set_hint(_, 0ULL));
        m_profile->exit(usr_region_id);
    }
}

TEST_F(KProfileTest, epoch)
{
    EXPECT_CALL(*m_record_log, epoch(_));
    m_profile->epoch();

}

TEST_F(KProfileTest, progress_multithread)
{
    EXPECT_CALL(*m_status, set_total_work_units(2, 5));
    EXPECT_CALL(*m_status, set_total_work_units(3, 6));
    m_profile->thread_init(2, 5);
    m_profile->thread_init(3, 6);
    {
        EXPECT_CALL(*m_status, increment_work_unit(3));
        EXPECT_CALL(*m_status, increment_work_unit(2));
        m_profile->thread_post(3);
        m_profile->thread_post(2);
    }
    {
        EXPECT_CALL(*m_status, increment_work_unit(3));
        m_profile->thread_post(3);
    }
}

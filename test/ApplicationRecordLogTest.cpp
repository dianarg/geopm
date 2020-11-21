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
#include "geopm_test.hpp"

#include "geopm_time.h"
#include "ApplicationRecordLog.hpp"
#include "record.hpp"
#include "MockSharedMemory.hpp"

using geopm::ApplicationRecordLog;
using geopm::SharedMemory;
using geopm::record_s;
using geopm::short_region_s;

class ApplicationRecordLogTest : public ::testing::Test
{
    protected:
        void SetUp();
        void will_lock(void);
        std::shared_ptr<MockSharedMemory> m_mock_shared_memory;
        std::unique_ptr<ApplicationRecordLog> m_record_log;

};

void ApplicationRecordLogTest::SetUp()
{
    size_t buffer_size = ApplicationRecordLog::buffer_size();
    m_mock_shared_memory = std::make_shared<MockSharedMemory>(buffer_size);
    m_record_log = ApplicationRecordLog::make_unique(m_mock_shared_memory);
}

void ApplicationRecordLogTest::will_lock()
{
    EXPECT_CALL(*m_mock_shared_memory, get_scoped_lock())
        .Times(1);
}

TEST_F(ApplicationRecordLogTest, empty_dump)
{
    std::vector<record_s> records;
    std::vector<short_region_s> short_regions;
    EXPECT_CALL(*m_mock_shared_memory, get_scoped_lock())
        .Times(1);
    m_record_log->dump(records, short_regions);
    EXPECT_EQ(0ULL, records.size());
    EXPECT_EQ(0ULL, short_regions.size());
}

TEST_F(ApplicationRecordLogTest, no_proc_set)
{
    EXPECT_CALL(*m_mock_shared_memory, get_scoped_lock())
        .Times(0);
    GEOPM_EXPECT_THROW_MESSAGE(m_record_log->enter(0,{{0,0}}),
                               GEOPM_ERROR_RUNTIME,
                               "set_process() must be called prior to calling enter(), exit() or epoch()");
    GEOPM_EXPECT_THROW_MESSAGE(m_record_log->exit(0,{{0,0}}),
                               GEOPM_ERROR_RUNTIME,
                               "set_process() must be called prior to calling enter(), exit() or epoch()");
    GEOPM_EXPECT_THROW_MESSAGE(m_record_log->epoch({{0,0}}),
                               GEOPM_ERROR_RUNTIME,
                               "set_process() must be called prior to calling enter(), exit() or epoch()");
}

TEST_F(ApplicationRecordLogTest, no_time_zero_set)
{
    EXPECT_CALL(*m_mock_shared_memory, get_scoped_lock())
        .Times(0);
    m_record_log->set_process(123);
    GEOPM_EXPECT_THROW_MESSAGE(m_record_log->enter(0,{{0,0}}),
                               GEOPM_ERROR_RUNTIME,
                               "set_time_zero() must be called prior to calling enter(), exit() or epoch()");
    GEOPM_EXPECT_THROW_MESSAGE(m_record_log->exit(0,{{0,0}}),
                               GEOPM_ERROR_RUNTIME,
                               "set_time_zero() must be called prior to calling enter(), exit() or epoch()");
    GEOPM_EXPECT_THROW_MESSAGE(m_record_log->epoch({{0,0}}),
                               GEOPM_ERROR_RUNTIME,
                               "set_time_zero() must be called prior to calling enter(), exit() or epoch()");
}

TEST_F(ApplicationRecordLogTest, one_entry)
{
    std::vector<record_s> records;
    std::vector<short_region_s> short_regions;
    int proc_id = 123;
    uint64_t hash = 0x1234abcd;
    geopm_time_s time_0 = {{1, 0}};
    geopm_time_s time = {{2, 0}};

    m_record_log->set_process(proc_id);
    m_record_log->set_time_zero(time_0);

    {
        EXPECT_CALL(*m_mock_shared_memory, get_scoped_lock())
            .Times(1);
        m_record_log->enter(hash, time);
    }

    {
        EXPECT_CALL(*m_mock_shared_memory, get_scoped_lock())
            .Times(1);
        m_record_log->dump(records, short_regions);
    }
    EXPECT_EQ(0ULL, short_regions.size());
    ASSERT_EQ(1ULL, records.size());
    EXPECT_EQ(1.0, records[0].time);
    EXPECT_EQ(proc_id, records[0].process);
    EXPECT_EQ(geopm::EVENT_REGION_ENTRY, records[0].event);
    EXPECT_EQ(hash, records[0].signal);
}

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
};

void ApplicationRecordLogTest::SetUp()
{
    size_t buffer_size = ApplicationRecordLog::buffer_size();
    m_mock_shared_memory = std::make_shared<MockSharedMemory>(buffer_size);
}

void ApplicationRecordLogTest::will_lock()
{
    EXPECT_CALL(m_mock_shared_memory, get_scoped_lock(void))
        .WillOnce(Return(nullptr));
}

TEST_F(ApplicationRecordLogTest, empty_dump)
{
    std::unique_ptr<ApplicationRecordLog> record_log =
        ApplicationRecordLog::record_log(m_mock_shared_memory);

    std::vector<record_s> records;
    std::vector<short_region_s> short_regions;
    will_lock();
    record_log->dump(records, regions);
    EXPECT_EQ(0ULL, records.size());
    EXPECT_EQ(0ULL, short_regions.size());
}

TEST_F(ApplicationRecordLogTest, no_proc_set)
{
    std::unique_ptr<ApplicationRecordLog> record_log =
        ApplicationRecordLog::record_log(m_mock_shared_memory);

    will_lock();
    GEOPM_EXPECT_THROW_MESSAGE(record_log.enter(0,{{0,0}}),
                               "set_process() must be called prior to calling enter()");
    will_lock();
    GEOPM_EXPECT_THROW_MESSAGE(record_log.exit(0,{{0,0}}),
                               "set_process() must be called prior to calling exit()");
    will_lock();
    GEOPM_EXPECT_THROW_MESSAGE(record_log.epoch({{0,0}}),
                               "set_process() must be called prior to calling epoch()");
}

TEST_F(ApplicationRecordLogTEst, no_time_zero_set)
{
    std::unique_ptr<ApplicationRecordLog> record_log =
        ApplicationRecordLog::record_log(m_mock_shared_memory);

    record_log->set_process(123);
    will_lock();
    GEOPM_EXPECT_THROW_MESSAGE(record_log.enter(0,{{0,0}}),
                               "set_zero_time() must be called prior to calling enter()");
    will_lock();
    GEOPM_EXPECT_THROW_MESSAGE(record_log.exit(0,{{0,0}}),
                               "set_zero_time() must be called prior to calling exit()");
    will_lock();
    GEOPM_EXPECT_THROW_MESSAGE(record_log.epoch({{0,0}}),
                               "set_zero_time() must be called prior to calling epoch()");
}

TEST_F(ApplicationRecordLogTest, one_entry)
{
    std::unique_ptr<ApplicationRecordLog> record_log =
        ApplicationRecordLog::record_log(m_mock_shared_memory);

    std::vector<record_s> records;
    std::vector<short_region_s> short_regions;
    will_lock();
    record_log->dump(records, regions);
    EXPECT_EQ(0ULL, records.size());
    EXPECT_EQ(0ULL, short_regions.size());
    will_lock();
    int proc_id = 123;
    uint64_t hash = 0x1234abcd;
    geopm_time_s time_0 = {{1, 0}};
    geopm_time_s time = {{2, 0}};
    record_log->set_process(proc_id);
    record_log->set_zero_time(time_0);
    will_lock();
    record_log->enter(hash, time);
    will_lock();
    record_log->dump(records, regions);
    EXPECT_EQ(0ULL, short_regions.size());
    ASSERT_EQ(1ULL, records.size());
    EXPECT_EQ(records[0].time, 1.0);
    EXPECT_EQ(records[0].process = proc_id);
    EXPECT_EQ(records[0].event = EVENT_REGION_ENTRY);
    EXPECT_EQ(records[0].signal = hash);
}

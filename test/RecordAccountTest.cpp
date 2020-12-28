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

#include <memory>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "RecordAccount.hpp"
#include "record.hpp"
#include "geopm_test.hpp"
#include "MockApplicationSampler.hpp"

using geopm::RecordAccount;
using geopm::RecordAccountImp;
using geopm::record_s;
using geopm::short_region_s;
using geopm::EVENT_REGION_ENTRY;
using geopm::EVENT_REGION_EXIT;
using geopm::EVENT_SHORT_REGION;
using testing::Return;

class RecordAccountTest : public ::testing::Test
{
    protected:
        void SetUp();

        MockApplicationSampler m_app_sampler;
        std::shared_ptr<RecordAccount> m_account;
};

void RecordAccountTest::SetUp()
{

    m_account = std::make_shared<RecordAccountImp>(m_app_sampler);
}

TEST_F(RecordAccountTest, entry_exit)
{
    std::vector<record_s> records;
    {
        // enter region
        records = {
            {1.0, 42, EVENT_REGION_ENTRY, 0xDADA}
        };
        m_app_sampler.inject_records(records);
        m_account->update();
        EXPECT_DOUBLE_EQ(0.0, m_account->get_process_region_runtime_average(0xDADA));
        EXPECT_DOUBLE_EQ(0.0, m_account->get_process_region_count_average(0xDADA));
    }
    {
        // exit region
        records = {
            {2.6, 42, EVENT_REGION_EXIT, 0xDADA}
        };
        m_app_sampler.inject_records(records);
        m_account->update();
        EXPECT_DOUBLE_EQ(1.6, m_account->get_process_region_runtime_average(0xDADA));
        EXPECT_DOUBLE_EQ(1.0, m_account->get_process_region_count_average(0xDADA));
    }
}

TEST_F(RecordAccountTest, short_region)
{
    std::vector<record_s> records;
    short_region_s short_region;

    {
        records = {
            {1.0, 42, EVENT_SHORT_REGION, 0}
        };
        short_region = {0xDADA, 2, 0.25};
        m_app_sampler.inject_records(records);
        EXPECT_CALL(m_app_sampler, get_short_region(0))
            .WillOnce(Return(short_region));
        m_account->update();
        EXPECT_DOUBLE_EQ(0.25, m_account->get_process_region_runtime_average(0xDADA));
        EXPECT_DOUBLE_EQ(2.0, m_account->get_process_region_count_average(0xDADA));
    }
    {
        records = {
            {2.0, 42, EVENT_SHORT_REGION, 0}
        };
        short_region = {0xDADA, 1, 0.05};
        m_app_sampler.inject_records(records);
        EXPECT_CALL(m_app_sampler, get_short_region(0))
            .WillOnce(Return(short_region));
        m_account->update();
        EXPECT_DOUBLE_EQ(0.30, m_account->get_process_region_runtime_average(0xDADA));
        EXPECT_DOUBLE_EQ(3.0, m_account->get_process_region_count_average(0xDADA));

    }
}

TEST_F(RecordAccountTest, multiple_processes)
{
    std::vector<record_s> records;
    short_region_s short_region;

    {
        // enter region
        records = {
            {1.1, 11, EVENT_REGION_ENTRY, 0xDADA},
            {1.2, 12, EVENT_REGION_ENTRY, 0xDADA},
            {1.3, 13, EVENT_REGION_ENTRY, 0xDADA},
            {1.4, 14, EVENT_REGION_ENTRY, 0xDADA},
        };
        m_app_sampler.inject_records(records);
        m_account->update();
        EXPECT_DOUBLE_EQ(0.0, m_account->get_process_region_runtime_average(0xDADA));
        EXPECT_DOUBLE_EQ(0.0, m_account->get_process_region_count_average(0xDADA));
        EXPECT_DOUBLE_EQ(0.0, m_account->get_process_region_runtime_average(0xBEAD));
        EXPECT_DOUBLE_EQ(0.0, m_account->get_process_region_count_average(0xBEAD));
    }
    {
        records = {
            {2.2, 11, EVENT_REGION_EXIT, 0xDADA},
            {2.4, 11, EVENT_SHORT_REGION, 0},
            {2.0, 12, EVENT_SHORT_REGION, 1},
            {2.0, 13, EVENT_SHORT_REGION, 2},
            {2.0, 14, EVENT_SHORT_REGION, 3},
            {2.8, 14, EVENT_REGION_EXIT, 0xDADA},
        };
        m_app_sampler.inject_records(records);
        short_region = {0xBEAD, 2, 0.15};
        EXPECT_CALL(m_app_sampler, get_short_region(0))
            .WillOnce(Return(short_region));
        short_region = {0xBEAD, 2, 0.25};
        EXPECT_CALL(m_app_sampler, get_short_region(1))
            .WillOnce(Return(short_region));
        short_region = {0xBEAD, 1, 0.35};
        EXPECT_CALL(m_app_sampler, get_short_region(2))
            .WillOnce(Return(short_region));
        short_region = {0xBEAD, 1, 0.45};
        EXPECT_CALL(m_app_sampler, get_short_region(3))
            .WillOnce(Return(short_region));

        m_account->update();
        // average of procs 11 and 14; 12 and 13 have not exited so they are 0
        // TODO: what is expected here?
        EXPECT_DOUBLE_EQ((1.1 + 1.4)/4.0, m_account->get_process_region_runtime_average(0xDADA));
        EXPECT_DOUBLE_EQ(2.0 / 4, m_account->get_process_region_count_average(0xDADA));

        // average of all procs for short region
        EXPECT_DOUBLE_EQ((0.15 + 0.25 + 0.35 + 0.45)/4.0,
                  m_account->get_process_region_runtime_average(0xBEAD));
        EXPECT_DOUBLE_EQ(6/4.0, m_account->get_process_region_count_average(0xBEAD));
    }
    {
        records = {
            {3.2, 12, EVENT_REGION_EXIT, 0xDADA},
            {3.3, 13, EVENT_REGION_EXIT, 0xDADA},
            {2.0, 12, EVENT_SHORT_REGION, 0},
            {2.0, 13, EVENT_SHORT_REGION, 1},
        };
        m_app_sampler.inject_records(records);
        short_region = {0xBEAD, 1, 0.15};
        EXPECT_CALL(m_app_sampler, get_short_region(0))
            .WillOnce(Return(short_region));
        short_region = {0xBEAD, 2, 0.25};
        EXPECT_CALL(m_app_sampler, get_short_region(1))
            .WillOnce(Return(short_region));
        m_account->update();

        // average of all procs
        EXPECT_DOUBLE_EQ((1.1 + 2.0 + 2.0 + 1.4)/4.0,
                  m_account->get_process_region_runtime_average(0xDADA));
        EXPECT_DOUBLE_EQ(1.0, m_account->get_process_region_count_average(0xDADA));

        EXPECT_DOUBLE_EQ((0.15 + 0.25 + 0.35 + 0.45 + 0.15 + 0.25)/4.0,
                  m_account->get_process_region_runtime_average(0xBEAD));
        EXPECT_DOUBLE_EQ((6+3)/4.0, m_account->get_process_region_count_average(0xBEAD));
    }

}

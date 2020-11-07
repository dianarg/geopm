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

#include "SSTSignal.hpp"
#include "MockSSTTransaction.hpp"
#include "geopm_hash.h"

using geopm::SSTSignal;
using testing::Return;
using testing::_;

class SSTTest : public :: testing :: Test
{
    protected:
        void SetUp(void);
        void TearDown(void);
        std::shared_ptr<MockSSTTransaction> m_trans;
        int m_num_cpu = 4;
};

void SSTTest::SetUp(void)
{
    m_trans = std::make_shared<MockSSTTransaction>();
}

void SSTTest::TearDown(void)
{

}

TEST_F(SSTTest, mailbox_read_batch)
{
    // TODO: multiple cpu
    int cpu = 3;
    uint32_t command = 0x7f;
    uint32_t subcommand = 0x33;
    uint32_t sub_arg = 0x56;
    uint32_t interface_param = 0x93;

    SSTSignal sig {m_trans, cpu, command, subcommand, sub_arg,
                   interface_param};

    int batch_idx = 42;
    EXPECT_CALL(*m_trans, add_mbox_read(cpu, command, subcommand, sub_arg,
                                        interface_param))
        .WillOnce(Return(batch_idx));

    sig.setup_batch();

    double expected = 6;
    EXPECT_CALL(*m_trans, sample(batch_idx)).WillOnce(Return(geopm_signal_to_field(expected)));

    double result = sig.sample();
    EXPECT_EQ(expected, result);
}

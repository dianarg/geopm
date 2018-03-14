/*
 * Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
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
#include <vector>
#include <utility>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "TreeComm.hpp"
#include "TreeCommLevel.hpp"
#include "MockComm.hpp"
#include "MockTreeCommLevel.hpp"
#include "geopm_test.hpp"
#include "config.h"

using geopm::ITreeCommLevel;
using geopm::TreeComm;
using geopm::IComm;
using testing::_;

class TreeCommTest : public ::testing::Test
{
    protected:
        void SetUp();

        std::shared_ptr<MockComm> m_mock_comm;
        std::vector<int> m_fan_out;
        std::vector<MockTreeCommLevel *> m_level_ptr;
        std::unique_ptr<TreeComm> m_tree_comm;
};

void TreeCommTest::SetUp()
{
    m_mock_comm = std::make_shared<MockComm>();
    m_fan_out = {2, 3, 4, 5};
    std::vector<std::unique_ptr<ITreeCommLevel> > temp;
    for (size_t lvl = 0; lvl < m_fan_out.size() + 1; ++lvl) {
        m_level_ptr.push_back(new MockTreeCommLevel);
        temp.emplace_back(m_level_ptr[lvl]);
    }

    //EXPECT_CALL(*m_mock_comm, split(_, _, _));
    EXPECT_CALL(*m_mock_comm, barrier());
    m_tree_comm.reset(new TreeComm(m_mock_comm, m_fan_out, 1, 1, std::move(temp)));
}

TEST_F(TreeCommTest, geometry)
{

    EXPECT_EQ(4, m_tree_comm->num_level());
    EXPECT_EQ(3, m_tree_comm->root_level());
    EXPECT_CALL(*(m_level_ptr[0]), level_rank());
    EXPECT_EQ(888, m_tree_comm->level_rank(0));
    EXPECT_CALL(*(m_level_ptr[1]), level_rank());
    EXPECT_EQ(888, m_tree_comm->level_rank(1));
    EXPECT_CALL(*(m_level_ptr[2]), level_rank());
    EXPECT_EQ(888, m_tree_comm->level_rank(2));
    EXPECT_CALL(*(m_level_ptr[3]), level_rank());
    EXPECT_EQ(888, m_tree_comm->level_rank(3));
    EXPECT_CALL(*(m_level_ptr[4]), level_rank());
    EXPECT_EQ(888, m_tree_comm->level_rank(4));


    // errors
    GEOPM_EXPECT_THROW_MESSAGE(m_tree_comm->level_rank(-1), GEOPM_ERROR_LEVEL_RANGE,
                               "level_rank");
    GEOPM_EXPECT_THROW_MESSAGE(m_tree_comm->level_rank(10), GEOPM_ERROR_LEVEL_RANGE,
                               "level_rank");

    GEOPM_EXPECT_THROW_MESSAGE(m_tree_comm->level_size(-1), GEOPM_ERROR_LEVEL_RANGE,
                               "level_size");
    GEOPM_EXPECT_THROW_MESSAGE(m_tree_comm->level_size(10), GEOPM_ERROR_LEVEL_RANGE,
                               "level_size");
}

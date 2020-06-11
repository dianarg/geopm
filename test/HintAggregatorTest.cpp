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

#include "geopm.h"
#include "HintAggregator.hpp"
#include "geopm_test.hpp"

using geopm::HintAggregator;

// shorter names for hints
enum {
    UNKNOWN = GEOPM_REGION_HINT_UNKNOWN,
    COMPUTE = GEOPM_REGION_HINT_COMPUTE,
    MEMORY = GEOPM_REGION_HINT_MEMORY,
    NETWORK = GEOPM_REGION_HINT_NETWORK,
    IO = GEOPM_REGION_HINT_SERIAL,
    PARALLEL = GEOPM_REGION_HINT_PARALLEL,
    IGNORE = GEOPM_REGION_HINT_IGNORE,
};

TEST(HintAggregatorTest, total_runtime)
{
    HintAggregator m_agg;

    EXPECT_EQ(0.0, m_agg.get_total_runtime(UNKNOWN));
    EXPECT_EQ(0.0, m_agg.get_total_runtime(COMPUTE));
    EXPECT_EQ(0.0, m_agg.get_total_runtime(IGNORE));

    m_agg.start(1.0);
    EXPECT_EQ(0.0, m_agg.get_total_runtime(UNKNOWN));
    EXPECT_EQ(0.0, m_agg.get_total_runtime(COMPUTE));
    EXPECT_EQ(0.0, m_agg.get_total_runtime(IGNORE));

    m_agg.change_hint(COMPUTE, 2.5);
    EXPECT_EQ(1.5, m_agg.get_total_runtime(UNKNOWN));
    EXPECT_EQ(0.0, m_agg.get_total_runtime(COMPUTE));
    EXPECT_EQ(0.0, m_agg.get_total_runtime(IGNORE));

    m_agg.stop(3.0);
    EXPECT_EQ(1.5, m_agg.get_total_runtime(UNKNOWN));
    EXPECT_EQ(0.5, m_agg.get_total_runtime(COMPUTE));
    EXPECT_EQ(0.0, m_agg.get_total_runtime(IGNORE));

    m_agg.start(3.5);
    m_agg.change_hint(IGNORE, 3.5);
    m_agg.stop(5.0);
    EXPECT_EQ(1.5, m_agg.get_total_runtime(UNKNOWN));
    EXPECT_EQ(0.5, m_agg.get_total_runtime(COMPUTE));
    EXPECT_EQ(1.5, m_agg.get_total_runtime(IGNORE));
}

TEST(HintAggregatorTest, errors)
{
    HintAggregator m_agg;

    // change hint should not be called if aggegregator hasn't started
    GEOPM_EXPECT_THROW_MESSAGE(m_agg.change_hint(UNKNOWN, 0), GEOPM_ERROR_RUNTIME,
                               "aggregator is not running");

    // can't call stop without start
    GEOPM_EXPECT_THROW_MESSAGE(m_agg.stop(0), GEOPM_ERROR_RUNTIME,
                               "aggregator is not running");

    // invalid hint
    m_agg.start(0);
    GEOPM_EXPECT_THROW_MESSAGE(m_agg.change_hint(UNKNOWN - 1, 0), GEOPM_ERROR_INVALID,
                               "invalid hint");
    GEOPM_EXPECT_THROW_MESSAGE(m_agg.get_total_runtime(UNKNOWN - 1), GEOPM_ERROR_INVALID,
                               "invalid hint");
}

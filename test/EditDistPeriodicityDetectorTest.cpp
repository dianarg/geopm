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

#include <iostream>

#include <cstdint>
#include "gtest/gtest.h"
#include "geopm_test.hpp"
#include "EditDistPeriodicityDetector.hpp"
#include "MockApplicationSampler.hpp"
#include "Helper.hpp"

using geopm::record_s;
using geopm::EditDistPeriodicityDetector;

class EditDistPeriodicityDetectorTest : public ::testing::Test
{
    protected:
        void SetUp();
        std::string m_test_root_path;
};


void EditDistPeriodicityDetectorTest::SetUp()
{
    m_test_root_path = "./test/";
}

void check_vals(std::string trace_file_path, int warmup, int period, int history_size=100, bool debug=false);
void check_vals(std::string trace_file_path, int start, int end, int period, int history_size, bool debug=false);
void check_vals(std::vector<record_s> recs, std::vector<std::vector<int> > expected, int history_size=100, bool debug=false);


/// Pattern 0: (A)x10
TEST_F(EditDistPeriodicityDetectorTest, pattern_a)
{
    int warmup = 1;
    int period = 1;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.0_pattern_a.trace", warmup, period, history_size);
}

/// Pattern 1: (AB)x15
TEST_F(EditDistPeriodicityDetectorTest, pattern_ab)
{
    int warmup = 3;
    int period = 2;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.1_pattern_ab.trace", warmup, period, history_size);
}

/// Pattern 2: (ABB)x12
TEST_F(EditDistPeriodicityDetectorTest, pattern_abb)
{
    int warmup = 5;
    int period = 3;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.2_pattern_abb.trace", warmup, period, history_size);
}

/// Pattern 3: (ABCDABCDABCDC) (ABCDABCDABCDABCDC)x6 (ABCD)
TEST_F(EditDistPeriodicityDetectorTest, pattern_abcdc)
{
    int warmup = 33;
    int period = 17;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.3_pattern_abcdc.trace", warmup, period, history_size);
}

/// Pattern 4: (AB) (ABABC)x3
TEST_F(EditDistPeriodicityDetectorTest, pattern_ababc)
{
    int warmup = 11;
    int period = 5;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.4_pattern_ababc.trace", warmup, period, history_size);
}

/// Pattern 5: (ABABABC)x6
TEST_F(EditDistPeriodicityDetectorTest, pattern_abababc)
{
    int warmup = 13;
    int period = 7;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.5_pattern_abababc.trace", warmup, period, history_size);
}

/// Pattern 6: (ABCD)x6 (E) (ABCD)x6
TEST_F(EditDistPeriodicityDetectorTest, pattern_add1)
{
    int period = 4;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.6_pattern_add1.trace", 7, 24, period, history_size);
    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.6_pattern_add1.trace", 32, period, history_size);
}

/// Pattern 7: (ABCD)x6 (EF) (ABCD)x9
TEST_F(EditDistPeriodicityDetectorTest, pattern_add2)
{
    int period = 4;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.7_pattern_add2.trace", 7, 24, period, history_size);
    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.7_pattern_add2.trace", 33, period, history_size);
}

/// Pattern 8: (ABCD)x6 (ABC) (ABCD)x12
TEST_F(EditDistPeriodicityDetectorTest, pattern_subtract1)
{
    int period = 4;
    int history_size = 100;

    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.8_pattern_subtract1.trace", 7, 27, period, history_size);
    check_vals(m_test_root_path + "EditDistPeriodicityDetectorTest.8_pattern_subtract1.trace", 54, period, history_size);
}

/// HELPER FUNCTIONS

/// start: inclusive
/// end: exclusive
void check_vals(std::string trace_file_path, int start, int end, int period, int history_size, bool debug)
{
    MockApplicationSampler app;
    app.inject_records(geopm::read_file(trace_file_path));

    std::vector<std::vector<int> > expected;

    for(int i = start; i > 0; i--) {
        expected.push_back({-1, 0});
    }

    std::vector<record_s> recs = app.get_records();

    int valid_recs = recs.size()-start;
    if(end-start < valid_recs) {
        valid_recs = end-start;
    }

    for(int i = valid_recs; i > 0; i--) {
        expected.push_back({period, 0});
    }

    check_vals(recs, expected, history_size, debug);
}

void check_vals(std::string trace_file_path, int warmup, int period, int history_size, bool debug)
{
    MockApplicationSampler app;
    app.inject_records(geopm::read_file(trace_file_path));

    std::vector<std::vector<int> > expected;

    for(int i = warmup; i > 0; i--) {
        expected.push_back({-1, 0});
    }

    std::vector<record_s> recs = app.get_records();

    int valid_recs = recs.size()-warmup;

    for(int i = valid_recs; i > 0; i--) {
        expected.push_back({period, 0});
    }

    check_vals(recs, expected, history_size, debug);
}

void check_vals(std::vector<record_s> recs, std::vector<std::vector<int> > expected, int history_size, bool debug)
{
    geopm::EditDistPeriodicityDetector edpd(history_size);

    for(size_t i = 0; i < expected.size(); i++) {
        edpd.update(recs[i]);
        int period = edpd.get_period();
        int score = edpd.get_score();

        if(debug) {
            std::cout << "Record: i=" << i << " period=" << period << " score=" << score << "\n";
            std::cout << "Expect: i=" << i << " period=" << expected[i][0] << " score=" << expected[i][1] << "\n";
        } else {
            if(expected[i][0] != -1) {
                // -1 means skip the test entry (probably for warmup).
                EXPECT_EQ(expected[i][0], edpd.get_period()) << "Record #: " << i;
                EXPECT_EQ(expected[i][1], edpd.get_score()) << "Record #: " << i;
            }
        }
    }
}

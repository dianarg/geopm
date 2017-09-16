/*
 * Copyright (c) 2015, 2016, 2017, Intel Corporation
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

#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sstream>
#include <utility>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "MSR.hpp"
#include "MockMSRIO.hpp"
#include "PlatformTopology.hpp"

using testing::_;
using testing::Invoke;
using testing::Sequence;

class MSRTest : public :: testing :: Test
{
    protected:
        void SetUp();
        void TearDown();

        int m_cpu_idx;
        MockMSRIO m_msrio;
        std::vector<const geopm::IMSR *> m_prog_msr;
        std::vector<std::string> m_prog_field_name;
        std::vector<double> m_prog_value;
        geopm::IMSR *m_msr;
        std::string m_name;
        uint64_t m_offset;
        std::vector<std::pair<std::string, struct geopm::IMSR::m_encode_s> > m_signals;
        std::vector<std::pair<std::string, struct geopm::IMSR::m_encode_s> > m_controls;
};

void MSRTest::SetUp()
{
    m_cpu_idx = 0;
    m_name = "test-msr";
    m_offset = 0xDEADBEEF;
    m_signals = {std::pair<std::string, struct geopm::IMSR::m_encode_s> ("sig1", {0, 8, 1.0}),
                 std::pair<std::string, struct geopm::IMSR::m_encode_s> ("sig2", {9, 16, 2.0})};
    m_controls = {std::pair<std::string, struct geopm::IMSR::m_encode_s> ("ctl1", {0, 8, 1.0}),
                 std::pair<std::string, struct geopm::IMSR::m_encode_s> ("ctl2", {9, 16, 2.0}),
                 std::pair<std::string, struct geopm::IMSR::m_encode_s> ("ctl3", {27, 56, 4.0})};
    geopm::MSR *stage0 = new geopm::MSR("stage0", 2, m_signals, m_controls);
    geopm::MSR *stage1 = new geopm::MSR("stage1", 8, m_signals, m_controls);
    geopm::MSR *stage2 = new geopm::MSR("stage2", 16, m_signals, m_controls);
    m_prog_msr = {stage0, stage1, stage2};
    m_prog_field_name = {"ctl1", "ctl2", "ctl3"};
    m_prog_value = {69.0, 72.0, 99.99};
}

void MSRTest::TearDown()
{
    for (auto msr_it = m_prog_msr.begin(); msr_it != m_prog_msr.end(); ++msr_it) {
        free((void *)(*msr_it));

    }
}

TEST_F(MSRTest, bang1)
{
    m_msr = new geopm::MSR(m_name, m_offset, m_signals, m_controls);
    std::string name;
    m_msr->name(name);

    EXPECT_EQ(m_name, name);
    EXPECT_EQ(m_offset, m_msr->offset());
    EXPECT_EQ(m_signals.size(), m_msr->num_signal());
    EXPECT_EQ(m_controls.size(), m_msr->num_control());
    EXPECT_EQ(geopm::GEOPM_DOMAIN_CPU, m_msr->domain_type());

    int idx = 0;
    for (auto signal_it = m_signals.begin(); signal_it != m_signals.end(); ++signal_it, idx++) {
        double value;
        uint64_t field, mask;
        m_msr->signal_name(idx, name);
        EXPECT_EQ((*signal_it).first, name);
        EXPECT_EQ(m_msr->signal_index((*signal_it).first), idx);
        value = m_msr->signal(idx, field);
        // EXPECTATION on value
    }

    idx = 0;
    for (auto control_it = m_controls.begin(); control_it != m_controls.end(); ++control_it, idx++) {
        double value = 1e-8;
        uint64_t field, mask;
        m_msr->control_name(idx, name);
        EXPECT_EQ((*control_it).first, name);
        EXPECT_EQ(m_msr->control_index((*control_it).first), idx);
        m_msr->control(idx, value, field, mask);
        //EXPECTATION on returned field and mask
    }

    idx = -1;
    EXPECT_THROW(m_msr->signal_name(idx, name), geopm::Exception);
    EXPECT_THROW(m_msr->control_name(idx, name), geopm::Exception);
    m_msr->program(0, m_cpu_idx, &m_msrio);
    delete m_msr;
}

TEST_F(MSRTest, bang2)
{
    int offset = 1010101;
    m_msr = new geopm::MSR(m_name, m_signals, m_prog_msr, m_prog_field_name, m_prog_value);
    std::string name;
    m_msr->name(name);

    Sequence s1;
    int idx = 0;
    for (auto msr_it = m_prog_msr.begin(); msr_it != m_prog_msr.end(); ++msr_it, idx++) {
        EXPECT_CALL(m_msrio, write_msr(m_cpu_idx, _, _, _))
            .InSequence(s1)
            .WillOnce(Invoke([this, msr_it] (int cpu_idx, uint64_t offset, uint64_t raw_value, uint64_t write_mask)
                {
                    EXPECT_EQ((*msr_it)->offset(), offset);
                }));
    }

    m_msr->program(offset, m_cpu_idx, &m_msrio);
    EXPECT_EQ(offset, m_msr->offset());
    delete m_msr;
}

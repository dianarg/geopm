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

#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <limits.h>
#include <sstream>
#include <fstream>
#include <string>
#include <map>
#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "geopm_sched.h"
//#include "PlatformIO.hpp"
//#include "PlatformIOInternal.hpp"
#include "PlatformTopo.hpp"
#include "MSRIO.hpp"
#include "Exception.hpp"
#include "MSRIOGroup.hpp"

using geopm::MSRIOGroup;

class MSRIOGroupTest : public :: testing :: Test
{
    protected:
        void SetUp();
        std::vector<std::string> m_test_dev_path;
        std::unique_ptr<geopm::MSRIOGroup> m_msrio_group;
};

class MockMSRIO : public geopm::MSRIO
{
    public:
        MockMSRIO();
        virtual ~MockMSRIO();
        std::vector<std::string> test_dev_paths();
    protected:
        void msr_path(int cpu_idx,
                      bool is_fallback,
                      std::string &path) override;
        void msr_batch_path(std::string &path) override;

        const size_t M_MAX_OFFSET;
        const int m_num_cpu;
        std::vector<std::string> m_test_dev_path;
};

MockMSRIO::MockMSRIO()
   : M_MAX_OFFSET(4096)
   , m_num_cpu(geopm_sched_num_cpu())
{
    union field_u {
        uint64_t field;
        uint16_t off[4];
    };
    union field_u fu;
    for (int cpu_idx = 0; cpu_idx < m_num_cpu; ++cpu_idx) {
        char tmp_path[NAME_MAX] = "/tmp/test_platform_io_dev_cpu_XXXXXX";
        int fd = mkstemp(tmp_path);
        if (fd == -1) {
           throw geopm::Exception("MockMSRIO: mkstemp() failed",
                                  errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        m_test_dev_path.push_back(tmp_path);

        int err = ftruncate(fd, M_MAX_OFFSET);
        if (err) {
            throw geopm::Exception("MockMSRIO: ftruncate() failed",
                                   errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        uint64_t *contents = (uint64_t *)mmap(NULL, M_MAX_OFFSET, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (contents == NULL) {
            throw geopm::Exception("MockMSRIO: mmap() failed",
                                   errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        close(fd);
        size_t num_field = M_MAX_OFFSET / sizeof(uint64_t);
        for (size_t field_idx = 0; field_idx < num_field; ++field_idx) {
            uint16_t offset = field_idx * sizeof(uint64_t);
            for (int off_idx = 0; off_idx < 4; ++off_idx) {
               fu.off[off_idx] = offset;
            }
            contents[field_idx] = fu.field;
        }
        munmap(contents, M_MAX_OFFSET);
    }
}

MockMSRIO::~MockMSRIO()
{
    for (auto &path : m_test_dev_path) {
        unlink(path.c_str());
    }
}

std::vector<std::string> MockMSRIO::test_dev_paths()
{
    return m_test_dev_path;
}

void MockMSRIO::msr_path(int cpu_idx,
                         bool is_fallback,
                         std::string &path)
{
    path = m_test_dev_path[cpu_idx];
}

void MockMSRIO::msr_batch_path(std::string &path)
{
    path = "test_dev_msr_safe";
}

void MSRIOGroupTest::SetUp()
{
     std::unique_ptr<MockMSRIO> msrio(new MockMSRIO);
     m_test_dev_path = msrio->test_dev_paths();
     m_msrio_group = std::unique_ptr<MSRIOGroup>(new MSRIOGroup(std::move(msrio), 0x657)); // KNL cpuid
}

template <typename Func> //, typename ...Args>
void ExpectGeopmErrorMessage(int err, std::string message, Func func) //, Args ...args)
{
    try {
        func();
        //func(std::forward<Args>(args)...);
        FAIL() << "Expected to fail, but succeeded.";
    }
    catch (const geopm::Exception &ex) {
        EXPECT_EQ(err, ex.err_value());
        EXPECT_THAT(ex.what(), ::testing::MatchesRegex(".*" + message + ".*"));
    }

}

// TODO: add whitelist function
/*
TEST_F(MSRIOGroupTest, whitelist)
{
    std::ifstream file("test/legacy_whitelist.out");
    std::string line;
    uint64_t offset;
    uint64_t  mask;
    std::string comment;
    std::map<uint64_t, uint64_t> legacy_map;
    std::map<uint64_t, uint64_t> curr_map;
    while (std::getline(file, line)) {
        if (line.compare(0, 1, "#") == 0) continue;
        std::string tmp;
        size_t sz;
        std::istringstream iss(line);
        iss >> tmp;
        offset = std::stoull(tmp, &sz, 16);
        iss >> tmp;
        mask = std::stoull(tmp, &sz, 16);
        iss >> comment;// #
        iss >> comment;// comment
        legacy_map[offset] = mask;
    }

    std::string whitelist = m_msrio_group->msr_whitelist();
    std::istringstream iss(whitelist);
    std::getline(iss, line);// throw away title line
    while (std::getline(iss, line)) {
        std::string tmp;
        size_t sz;
        std::istringstream iss(line);
        iss >> tmp;
        offset = std::stoull(tmp, &sz, 16);
        iss >> tmp;
        mask = std::stoull(tmp, &sz, 16);
        iss >> comment;// #
        iss >> comment;// comment
        curr_map[offset] = mask;
    }

    for (auto it = curr_map.begin(); it != curr_map.end(); ++it) {
        offset = it->first;
        mask = it->second;
        auto leg_it = legacy_map.find(offset);
        if (leg_it == legacy_map.end()) {
            //not found error
            if (!mask) {
                EXPECT_TRUE(false) << std::setfill('0') << std::hex << "new read offset 0x" << std::setw(8) << offset << " introduced";
            }
            continue;
        }
        uint64_t leg_mask = leg_it->second;
        EXPECT_EQ(mask, mask & leg_mask) << std::setfill('0') << std::hex << "offset 0x" << std::setw(8) << offset << "write mask change detected, from 0x"
            << std::setw(16) << leg_mask << " to 0x" << mask << " bitwise AND yields 0x" << (mask & leg_mask);
    }
}
*/


TEST_F(MSRIOGroupTest, freq_signal)
{
    ExpectGeopmErrorMessage(GEOPM_ERROR_NOT_IMPLEMENTED, "non-CPU domain_type", [this] {
            m_msrio_group->push_signal("PERF_STATUS:FREQ", 99, 0);
        });
    ExpectGeopmErrorMessage(GEOPM_ERROR_INVALID, "signal name.*not found", [this] {
            m_msrio_group->push_signal("INVALID", geopm::IPlatformTopo::M_DOMAIN_CPU, 0);
        });


    int fd = open(m_test_dev_path[0].c_str(), O_RDWR);
    ASSERT_NE(-1, fd);
    uint64_t value = 0xB00;
    size_t num_write = pwrite(fd, &value, sizeof(value), 0x198);
    ASSERT_EQ(num_write, sizeof(value));

    int idx = m_msrio_group->push_signal("PERF_STATUS:FREQ", geopm::IPlatformTopo::M_DOMAIN_CPU, 0);
    ASSERT_EQ(0, idx);

    ExpectGeopmErrorMessage(GEOPM_ERROR_RUNTIME, "sample.* called before signal was read", [this] {
            m_msrio_group->sample(0);
        });


    m_msrio_group->read_batch();
    double freq = m_msrio_group->sample(idx);
    EXPECT_EQ(1.1e9, freq);
    freq = m_msrio_group->sample(idx);

    // sample again without read should get same value
    freq = m_msrio_group->sample(idx);
    EXPECT_EQ(1.1e9, freq);

    value = 0xC00;
    num_write = pwrite(fd, &value, sizeof(value), 0x198);
    m_msrio_group->read_batch();
    freq = m_msrio_group->sample(idx);
    EXPECT_EQ(1.2e9, freq);

    ExpectGeopmErrorMessage(GEOPM_ERROR_INVALID, "cannot push a signal after read_batch", [this] {
            m_msrio_group->push_signal("PERF_STATUS:FREQ", geopm::IPlatformTopo::M_DOMAIN_CPU, 0);
        });
}

TEST_F(MSRIOGroupTest, freq_control)
{

    int fd = open(m_test_dev_path[0].c_str(), O_RDWR);
    ASSERT_NE(-1, fd);
    uint64_t value;
    size_t num_read;

    //// DRG: what is this for?
    num_read = pread(fd, &value, sizeof(value), 0x0);
    EXPECT_EQ(8ULL, num_read);
    EXPECT_EQ(0x0ULL, value);
    num_read = pread(fd, &value, sizeof(value), 0x198);
    EXPECT_EQ(8ULL, num_read);
    EXPECT_EQ(0x0198019801980198ULL, value);
    num_read = pread(fd, &value, sizeof(value), 0x1A0);
    EXPECT_EQ(8ULL, num_read);
    EXPECT_EQ(0x01A001A001A001A0ULL, value);
    ////


    ExpectGeopmErrorMessage(GEOPM_ERROR_NOT_IMPLEMENTED, "non-CPU domain_type", [this] {
            m_msrio_group->push_control("PERF_CTL:FREQ", 99, 0); });
    ExpectGeopmErrorMessage(GEOPM_ERROR_INVALID, "control name.*not found", [this] {
            m_msrio_group->push_control("INVALID", geopm::IPlatformTopo::M_DOMAIN_CPU, 0);
        });

    int idx = m_msrio_group->push_control("PERF_CTL:FREQ", geopm::IPlatformTopo::M_DOMAIN_CPU, 0);
    ASSERT_EQ(0, idx);
    // Set frequency to 1 GHz
    m_msrio_group->adjust(0, 1e9);
    m_msrio_group->write_batch();
    num_read = pread(fd, &value, sizeof(value), 0x199);
    EXPECT_EQ(8ULL, num_read);
    EXPECT_EQ(0xA00ULL, (value & 0xFF00));
    // Set frequency to 5 GHz
    m_msrio_group->adjust(0, 5e9);
    // Calling adjust without calling write_batch() should not
    // change the platform.
    num_read = pread(fd, &value, sizeof(value), 0x199);
    EXPECT_EQ(8ULL, num_read);
    EXPECT_EQ(0xA00ULL, (value & 0xFF00));
    m_msrio_group->write_batch();
    // Now that write_batch() been called the value on the platform
    // should be updated.
    num_read = pread(fd, &value, sizeof(value), 0x199);
    EXPECT_EQ(8ULL, num_read);
    EXPECT_EQ(0x3200ULL, (value & 0xFF00));
    close(fd);

    ExpectGeopmErrorMessage(GEOPM_ERROR_INVALID, "cannot push a control after .*adjust", [this] {
            m_msrio_group->push_control("INVALID", geopm::IPlatformTopo::M_DOMAIN_CPU, 0);
        });

}

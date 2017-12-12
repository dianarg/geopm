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

#include <unistd.h>
#include <fstream>
#include "gtest/gtest.h"

#include "PlatformTopo.hpp"

class PlatformTopoTest : public :: testing :: Test
{

};

TEST_F(PlatformTopoTest, laptop_num_domain)
{
    const std::string lscpu_str(
        "Architecture:          x86_64\n"
        "CPU op-mode(s):        32-bit, 64-bit\n"
        "Byte Order:            Little Endian\n"
        "CPU(s):                2\n"
        "On-line CPU(s) mask:   0x3\n"
        "Thread(s) per core:    1\n"
        "Core(s) per socket:    2\n"
        "Socket(s):             1\n"
        "NUMA node(s):          1\n"
        "Vendor ID:             GenuineIntel\n"
        "CPU family:            6\n"
        "Model:                 61\n"
        "Model name:            Intel(R) Core(TM) i7-5650U CPU @ 2.20GHz\n"
        "Stepping:              4\n"
        "CPU MHz:               2200.000\n"
        "BogoMIPS:              4400.00\n"
        "Hypervisor vendor:     KVM\n"
        "Virtualization type:   full\n"
        "L1d cache:             32K\n"
        "L1i cache:             32K\n"
        "L2 cache:              256K\n"
        "L3 cache:              4096K\n"
        "NUMA node0 CPU(s):     0x3\n");
    const std::string lscpu_file_name("PlatformTopoTest-lscpu_laptop");
    std::ofstream lscpu_fid(lscpu_file_name);
    lscpu_fid << lscpu_str;
    lscpu_fid.close();
    geopm::PlatformTopo topo(lscpu_file_name);
    EXPECT_EQ(1, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_BOARD));
    EXPECT_EQ(1, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_PACKAGE));
    EXPECT_EQ(2, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_CORE));
    EXPECT_EQ(2, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_CPU));
    EXPECT_EQ(1, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_BOARD_MEMORY));
    EXPECT_EQ(0, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_PACKAGE_MEMORY));
    unlink(lscpu_file_name.c_str());
}

TEST_F(PlatformTopoTest, knl_num_domain)
{
    const std::string lscpu_str(
        "Architecture:          x86_64\n"
        "CPU op-mode(s):        32-bit, 64-bit\n"
        "Byte Order:            Little Endian\n"
        "CPU(s):                256\n"
        "On-line CPU(s) mask:   0xffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n"
        "Thread(s) per core:    4\n"
        "Core(s) per socket:    64\n"
        "Socket(s):             1\n"
        "NUMA node(s):          2\n"
        "Vendor ID:             GenuineIntel\n"
        "CPU family:            6\n"
        "Model:                 87\n"
        "Model name:            Intel(R) Genuine Intel(R) CPU 0000 @ 1.30GHz\n"
        "Stepping:              1\n"
        "CPU MHz:               1030.402\n"
        "BogoMIPS:              2593.93\n"
        "L1d cache:             32K\n"
        "L1i cache:             32K\n"
        "L2 cache:              1024K\n"
        "NUMA node0 CPU(s):     0xffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff\n"
        "NUMA node1 CPU(s):     0x0\n");
    const std::string lscpu_file_name("PlatformTopoTest-lscpu_knl");
    std::ofstream lscpu_fid(lscpu_file_name);
    lscpu_fid << lscpu_str;
    lscpu_fid.close();
    geopm::PlatformTopo topo(lscpu_file_name);
    EXPECT_EQ(1, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_BOARD));
    EXPECT_EQ(1, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_PACKAGE));
    EXPECT_EQ(64, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_CORE));
    EXPECT_EQ(256, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_CPU));
    EXPECT_EQ(1, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_BOARD_MEMORY));
    EXPECT_EQ(1, topo.num_domain(geopm::IPlatformTopo::M_DOMAIN_PACKAGE_MEMORY));
    unlink(lscpu_file_name.c_str());
}

            // How do we ask how many sockets per board?
            //    num_domain(M_DOMAIN_BOARD, M_DOMAIN_PACKAGE)
            // How do we ask how many CPUs per socket?
            //    num_domain(M_DOMAIN_PACKAGE, M_DOMAIN_CPU)
            // How do we know which Linux logical CPUs are on core 5?
            //    domain_cpus(M_DOMAIN_CORE, 5, cpu_idx_set);
            // How do we ask if there is on package memory?
            //    num_domain(M_DOMAIN_PACKAGE, M_DOMAIN_PACKAGE_MEMORY) > 0
            // How do we ask if the frequency control is per package or per core?
            //    platform_io().control_domain_type("PERF_CTL:FREQ") == M_DOMAIN_PACKAGE
            // How do we ask which socket Linux logical CPU 8 is on?
            //    domain_idx(M_DOMAIN_PACKAGE, 8)
            // How do we find out all of the other Linux logical CPUs that share a socket with CPU 8?
            //    domain_cpus(M_DOMAIN_PACKAGE, domain_idx(M_DOMAIN_PACKAGE, 8), socket_cpu_set)
            // How do we define a group all linux logical CPUs that are divisable by 4?
            //    int num_cpu = num_domain(M_DOMAIN_CPU);
            //    for (int i = 0; i < num_cpu; i +=4) {
            //        domain_idx.push_back(i);
            //    }
            //    uint64_t group_domain = group_ext_define(0, domain_idx);

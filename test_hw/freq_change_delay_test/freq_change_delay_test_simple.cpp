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

#include <stdlib.h>
#include <iostream>
#include <stdint.h>
#include <sched.h>
#include <unistd.h>

#include "geopm_time.h"
#include "MSRIO.hpp"

/// macro definitions
#define IA_32_PERF_STATUS_MSR   0x198
#define IA_32_PERF_CTL_MSR      0x199
#define IA_32_PERF_MASK         0xFF00
#define NUM_TRIAL               600

geopm::IMSRIO &msrio(void){
    static geopm::MSRIO instance;
    return instance;
}

int write_all_cpu(uint64_t write_value)
{
    static __thread int cpu_idx = -1;
    if (cpu_idx == -1) {
        #pragma omp parallel
        {
            cpu_idx = sched_getcpu();
        }
    }
    #pragma omp parallel
    {
        msrio().write_msr(cpu_idx, IA_32_PERF_CTL_MSR, write_value, IA_32_PERF_MASK);
    }
    return 0;
}

bool read_all_cpu(uint64_t target_val)
{
    static __thread int cpu_idx = -1;
    bool result = true;

    if (cpu_idx == -1) {
        #pragma omp parallel
        {
            cpu_idx = sched_getcpu();
        }
    }
    #pragma omp parallel shared(result)
    {
        uint64_t read_val = msrio().read_msr(cpu_idx, IA_32_PERF_STATUS_MSR);
        if (read_val & IA_32_PERF_MASK != target_val) {
            result = false;
        }
    }
    return result;
}

int main(int argc, char **argv)
{
    const char *usage = "Usage: %s FREQ_0 FREQ_1\nFrequency in units of Hz\n\n";
    if (argc != 3) {
        fprintf(stderr, usage, argv[0]);
        return -1;
    }

    const long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    const uint64_t freq0 = (uint64_t)(atof(argv[1]) / 1e8) << 8;
    const uint64_t freq1 = (uint64_t)(atof(argv[2]) / 1e8) << 8;

    struct geopm_time_s time0 = {{0, 0}};
    struct geopm_time_s time1 = {{0, 0}};
    struct geopm_time_s time2 = {{0, 0}};
    struct geopm_time_s time3 = {{0, 0}};

    for (int loop_idx = 0; loop_idx < NUM_TRIAL; ++loop_idx) {
        uint64_t freq = loop_idx % 2 ? freq0 : freq1;
        write_all_cpu(freq);
        geopm_time(&time0);
        bool is_done = false;
        double delay = 0.0;
        while (!is_done && delay < 1.0) {
            geopm_time(&time1);
            is_done = read_all_cpu(freq);
            do {
                geopm_time(&time2);
                sched_yield();
                geopm_time(&time3);
                double yield_time = geopm_time_diff(&time2, &time3);
                if (yield_time > 4e-6) {
                    std::cerr << "Warning: yeild time " << yield_time << std::endl;
                }
            } while (geopm_time_diff(&time1, &time2) < 5e-6);
            delay = geopm_time_diff(&time0, &time2);
        }
        if (is_done) {
            std::cout << delay << std::endl;
            while (delay < 1.0) {
                geopm_time(&time1);
                delay = geopm_time_diff(&time0, &time1);
                sched_yield();
            }
        }
        else {
            std::cout << "FAILED" << std::endl;
        }
    }
    return 0;
}

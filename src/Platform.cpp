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

#include <set>
#include <string>
#include <inttypes.h>
#include <cpuid.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdexcept>

#include "Exception.hpp"
#include "Platform.hpp"
#include "PlatformFactory.hpp"
#include "geopm_message.h"
#include "config.h"

extern "C"
{
    int geopm_platform_msr_save(const char *path)
    {
        int err = 0;
        try {
            geopm::PlatformFactory platform_factory;
            geopm::Platform *platform = platform_factory.platform("rapl", true);
            platform->save_msr_state(path);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }

        return err;
    }

    int geopm_platform_msr_restore(const char *path)
    {
        int err = 0;

        try {
            geopm::PlatformFactory platform_factory;
            geopm::Platform *platform = platform_factory.platform("rapl", true);
            platform->restore_msr_state(path);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }

        return err;
    }

    int geopm_platform_msr_whitelist(FILE *file_desc)
    {
        int err = 0;
        try {
            geopm::PlatformFactory platform_factory;
            geopm::Platform *platform = platform_factory.platform("rapl", false);

            platform->write_msr_whitelist(file_desc);
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }

        return err;
    }
}

namespace geopm
{
    Platform::Platform()
        : m_imp(NULL)
        , m_num_rank(0)
    {

    }

    Platform::~Platform()
    {

    }

    void Platform::set_implementation(PlatformImp* platform_imp, bool do_initialize)
    {
        m_imp = platform_imp;
        if (do_initialize) {
            m_imp->initialize();
            initialize();
        }
    }

    void Platform::name(std::string &plat_name) const
    {
        if (m_imp == NULL) {
            throw Exception("Platform implementation is missing", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        plat_name = m_imp->platform_name();
    }

    void Platform::transform_rank_data(uint64_t region_id, const struct geopm_time_s &aligned_time,
                                       const std::vector<double> &aligned_data,
                                       std::vector<struct geopm_telemetry_message_s> &telemetry)
    {
/*        int num_cpu = m_imp->num_logical_cpu();
        int num_platform_signal = m_imp->num_energy_signal() +
                                  m_imp->num_counter_signal();

        std::vector<double> runtime(m_num_domain);
        std::vector<double> min_progress(m_num_domain);
        std::vector<double> max_progress(m_num_domain);

        std::fill(runtime.begin(), runtime.end(), -DBL_MAX);
        std::fill(min_progress.begin(), min_progress.end(), DBL_MAX);
        std::fill(max_progress.begin(), max_progress.end(), -DBL_MAX);

        int num_cpu_per_domain = num_cpu / m_num_domain;
        int rank_offset = m_num_domain * num_platform_signal;
        int local_rank = 0; // Rank index local to the compute node
        for (size_t i = rank_offset;  i < aligned_data.size(); i += GEOPM_NUM_TELEMETRY_TYPE_RANK) {
            for (auto it = m_rank_cpu[local_rank].begin(); it != m_rank_cpu[local_rank].end(); ++it) {
                if (aligned_data[i + 1] != -1.0) { // Check runtime to see if the sample is valid
                    int off = (*it) / num_cpu_per_domain;
                    // Find minimum progress for any rank on the package
                    if (aligned_data[i] < min_progress[off]) {
                        min_progress[off] = aligned_data[i];
                    }
                    // Find maximum progress for any rank on the package
                    if (aligned_data[i] > max_progress[off]) {
                         max_progress[off] = aligned_data[i];
                    }
                    // Find maximum runtime for any rank on the package
                    if (aligned_data[i + 1] > runtime[off]) {
                        runtime[off] = aligned_data[i + 1];
                    }
                }
            }
            ++local_rank;
        }
        // Insert platform signals
        for (int i = 0; i < rank_offset; ++i) {
            int domain_idx = i / num_platform_signal;
            int signal_idx = i % num_platform_signal;
            telemetry[domain_idx].signal[signal_idx] = aligned_data[domain_idx * num_platform_signal + signal_idx];
        }
        // Insert application signals
        int domain_idx = 0;
        for (int i = 0; i < m_num_domain * GEOPM_NUM_TELEMETRY_TYPE_RANK; i += GEOPM_NUM_TELEMETRY_TYPE_RANK) {
            // Do not drop a region exit
            if (max_progress[domain_idx] == 1.0) {
                telemetry[domain_idx].signal[num_platform_signal] = 1.0;
            }
            else {
                telemetry[domain_idx].signal[num_platform_signal] = min_progress[domain_idx] == DBL_MAX ? 0.0 : min_progress[domain_idx];
            }
            telemetry[domain_idx].signal[num_platform_signal + 1] = runtime[domain_idx] == -DBL_MAX ? -1.0 : runtime[domain_idx];
            ++domain_idx;
        }
        // Insert region and timestamp
        for (int i = 0; i < m_num_domain; ++i) {
            telemetry[i].region_id = region_id;
            telemetry[i].timestamp = aligned_time;
        }*/
    }

    void Platform::init_transform(const std::vector<int> &cpu_rank)
    {
/*
        std::set<int> rank_set;
        for (auto it = cpu_rank.begin(); it != cpu_rank.end(); ++it) {
            rank_set.insert(*it);
        }
        m_num_rank = rank_set.size();
        int i = 0;
        std::map<int, int> rank_map;
        for (auto it = rank_set.begin(); it != rank_set.end(); ++it) {
            rank_map.insert(std::pair<int, int>(*it, i));
            ++i;
        }
        m_rank_cpu.resize(m_num_rank);
        for (i = 0; i < (int)cpu_rank.size(); ++i) {
            m_rank_cpu[rank_map.find(cpu_rank[i])->second].push_back(i);
        }*/
    }

    int Platform::num_domain(void)
    {
        if (!m_num_domain && m_imp) {
            // Of all available control methods, set m_num_domain to the
            // finest granularity of control.
            for (int ctl_type = 0; ctl_type < GEOPM_NUM_CONTROL_TYPE; ++ctl_type) {
                int num_domain = m_imp->num_control_domain(ctl_type);
                if (num_domain > m_num_domain) {
                   m_num_domain = num_domain;
                }
            }
        }
        return m_num_domain;
    }

    void Platform::save_msr_state(const char *path) const
    {
        m_imp->save_msr_state(path);
    }

    void Platform::restore_msr_state(const char *path) const
    {
        m_imp->restore_msr_state(path);
    }

    void Platform::write_msr_whitelist(FILE *file_desc) const
    {
        if (file_desc == NULL) {
            throw Exception("Platform(): file descriptor is NULL", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        m_imp->whitelist(file_desc);
    }

    void Platform::revert_msr_state(void) const
    {
        m_imp->revert_msr_state();
    }

    double Platform::control_latency_ms(int control_type) const
    {
        return m_imp->control_latency_ms(control_type);
    }

    double Platform::throttle_limit_mhz(void) const
    {
        return m_imp->throttle_limit_mhz();
    }

    bool Platform::is_updated(void)
    {
        return m_imp->is_updated();
    }
}

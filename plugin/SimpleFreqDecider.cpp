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

#include <hwloc.h>
#include <algorithm>
#include <thread>

#include "geopm.h"
#include "geopm_message.h"
#include "geopm_plugin.h"
#include "geopm_sched.h"

#include "SimpleFreqDecider.hpp"
#include "GoverningDecider.hpp"
#include "Exception.hpp"
#include <fstream>
#include <string>
#include <cmath>

#include "Region.hpp"

#include <iostream>
#include <stdlib.h>

int geopm_plugin_register(int plugin_type, struct geopm_factory_c *factory, void *dl_ptr)
{
    int err = 0;

    try {
        if (plugin_type == GEOPM_PLUGIN_TYPE_DECIDER) {
            geopm::IDecider *decider = new geopm::SimpleFreqDecider;
            geopm_factory_register(factory, decider, dl_ptr);
        }
    }
    catch (...) {
        err = geopm::exception_handler(std::current_exception());
    }

    return err;
}

namespace geopm
{
    class SimpleFreqRegion
    {
        public:
            SimpleFreqRegion(IRegion *region, double freq_min, double freq_max, double freq_step);
            virtual ~SimpleFreqRegion();
            double freq(void);
            void update(void);
        protected:
            const size_t m_max_increase;
            const size_t m_num_freq;
            IRegion *m_region;
            size_t m_num_increase;
            int m_is_learning;
            double m_target;
            size_t m_curr_idx;
            std::vector<double> m_freq;
            std::vector<double> m_bandwidth;
            std::vector<size_t> m_num_sample;
    };

    SimpleFreqRegion::SimpleFreqRegion(IRegion *region, double freq_min, double freq_max, double freq_step)
        : m_max_increase(4)
        , m_num_freq(1 + (size_t)ceil((freq_max - freq_min) / freq_step))
        , m_region(region)
        , m_num_increase(0)
        , m_is_learning(true)
        , m_target(0.0)
        , m_curr_idx(m_num_freq - 1)
        , m_freq(m_num_freq)
        , m_bandwidth(m_num_freq, 0)
        , m_num_sample(m_num_freq, 0)
    {
        double freq = freq_min;
        for (auto &freq_it : m_freq) {
            freq_it = freq;
            freq += freq_step;
        }
    }

    SimpleFreqRegion::~SimpleFreqRegion()
    {

    }

    double SimpleFreqRegion::freq(void)
    {
        return m_freq[m_curr_idx];
    }

    void SimpleFreqRegion::update(void)
    {
        if (m_is_learning) {
            double bandwidth = m_region->derivative(0, GEOPM_TELEMETRY_TYPE_READ_BANDWIDTH);
            if (!isnan(bandwidth)) {
                // Update upper and lower bounds on bandwidth for this frequency.
                m_bandwidth[m_curr_idx] += bandwidth;
                ++(m_num_sample[m_curr_idx]);
            }
            if (m_num_sample[m_curr_idx]) {
                double curr_bw = m_bandwidth[m_curr_idx] / m_num_sample[m_curr_idx];
std::cerr << "m_bandwidth = " << m_bandwidth[m_curr_idx] << "m_num_sample = " << m_num_sample[m_curr_idx] << " curr_bw = " << curr_bw << std::endl;
                if (m_curr_idx == m_num_freq - 1 &&
                    m_num_sample[m_curr_idx] >= m_max_increase) {
                    // Set the target if we are at max freq and we have
                    // made enough samples.
                    m_target = 0.9 * curr_bw;
                }
                if (m_target != 0.0) {
                    // Once we have made enough measurements of the bandwidth
                    // at max frequency start to explore lower frequencies.
                    if (curr_bw < m_target) {
                        ++m_curr_idx;
                        ++m_num_increase;
                        if (m_num_increase == m_max_increase) {
                            // If the frequency has been lowered too far too
                            // many times stop learning.
                            m_is_learning = false;
                        }
                    }
                    else if (m_curr_idx != 0 &&
                             curr_bw > m_target) {
                        // Check if the bandwidth is within 10% of the optimal
                        // bandwidth, and if so, lower the frequency.
                        --m_curr_idx;
                    }
                }
            }
        }
    }

    SimpleFreqDecider::SimpleFreqDecider()
        : GoverningDecider()
        , m_cpu_info_path("/proc/cpuinfo")
        , m_cpu_freq_min_path("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq")
        , m_cpu_freq_max_path("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq")
        , m_freq_min(cpu_freq_min())
        , m_freq_max(cpu_freq_max())
        , m_freq_step(100e6)
        , m_region_last(NULL)
    {
        m_name = "simple_freq";
    }

    SimpleFreqDecider::SimpleFreqDecider(const SimpleFreqDecider &other)
        : GoverningDecider(other)
        , m_cpu_info_path(other.m_cpu_info_path)
        , m_cpu_freq_min_path(other.m_cpu_freq_min_path)
        , m_cpu_freq_max_path(other.m_cpu_freq_max_path)
        , m_freq_min(other.m_freq_min)
        , m_freq_max(other.m_freq_max)
        , m_freq_step(other.m_freq_step)
        , m_region_last(other.m_region_last)
    {
        for (const auto &it : other.m_region_map) {
            m_region_map.insert(std::pair<uint64_t, SimpleFreqRegion *>(it.first, new SimpleFreqRegion(*(it.second))));
        }
    }

    SimpleFreqDecider::~SimpleFreqDecider()
    {
        for (auto &it : m_region_map) {
            delete it.second;
        }
    }

    IDecider *SimpleFreqDecider::clone(void) const
    {
        return (IDecider*)(new SimpleFreqDecider(*this));
    }

    bool SimpleFreqDecider::update_policy(IRegion &curr_region, IPolicy &curr_policy)
    {
        if (m_region_last != NULL &&
            m_region_last->identifier() != curr_region.identifier()) {
            // We have just left a region, so update it.
            auto region_it = m_region_map.find(m_region_last->identifier());
            if (region_it == m_region_map.end()) {
                auto tmp_it = m_region_map.insert(std::pair<uint64_t, SimpleFreqRegion *>(
                              m_region_last->identifier(), new SimpleFreqRegion(
                              m_region_last, m_freq_min, m_freq_max, m_freq_step)));
                region_it = tmp_it.first;
            }
            (*region_it).second->update();
            // Set the frequency for the current region.
            region_it = m_region_map.find(curr_region.identifier());
            if (region_it == m_region_map.end()) {
                auto tmp_it = m_region_map.insert(std::pair<uint64_t, SimpleFreqRegion *>(
                              m_region_last->identifier(), new SimpleFreqRegion(
                              m_region_last, m_freq_min, m_freq_max, m_freq_step)));
                region_it = tmp_it.first;
            }
            curr_policy.ctl_cpu_freq(std::vector<double>(geopm_sched_num_cpu(), (*region_it).second->freq()));
        }
        m_region_last = &curr_region;

        return GoverningDecider::update_policy(curr_region, curr_policy);
    }

    double SimpleFreqDecider::cpu_freq_sticker(void)
    {
        double result = NAN;
        const std::string key = "model name\t:";
        std::ifstream cpuinfo_file(m_cpu_info_path);
        if (!cpuinfo_file.is_open()) {
            throw Exception("SimpleFreqDecider::cpu_freq_sticker(): unable to open " + m_cpu_info_path,
                            errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        while (isnan(result) && cpuinfo_file.good()) {
            std::string line;
            getline(cpuinfo_file, line);
            if (line.find(key) != std::string::npos) {
                size_t at_pos = line.find("@");
                size_t ghz_pos = line.find("GHz");
                if (at_pos != std::string::npos &&
                    ghz_pos != std::string::npos) {
                    try {
                        result = 1e9 * std::stod(line.substr(at_pos + 1, ghz_pos));
                    }
                    catch (std::invalid_argument) {

                    }
                }
            }
        }
        cpuinfo_file.close();
        if (isnan(result)) {
            throw Exception("SimpleFreqDecider::cpu_freq_sticker(): unable to parse sticker frequency from " + m_cpu_info_path,
                            errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        return result;
    }

    double SimpleFreqDecider::cpu_freq_min(void)
    {
        double result = NAN;
        const char* env_simple_freq_max = getenv("GEOPM_SIMPLE_FREQ_MAX");
        if (env_simple_freq_max) {
             try {
                 result = std::stod(env_simple_freq_max);
             }
             catch (std::invalid_argument) {

             }
        }
        if (isnan(result)) {
            std::ifstream freq_file(m_cpu_freq_min_path);
            if (freq_file.is_open()) {
                std::string line;
                getline(freq_file, line);
                try {
                    result = 1e4 * std::stod(line);
                }
                catch (std::invalid_argument) {

                }
            }
        }
        if (isnan(result)) {
            try {
                result = cpu_freq_sticker() - 6 * m_freq_step;
            }
            catch (Exception) {

            }
        }
        if (isnan(result)) {
            throw Exception("SimpleFreqDecider::cpu_freq_min(): unable to parse minimum frequency",
                            errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        return result;
    }

    double SimpleFreqDecider::cpu_freq_max(void)
    {
        double result = NAN;
        const char* env_simple_freq_max = getenv("GEOPM_SIMPLE_FREQ_MAX");
        if (env_simple_freq_max) {
             try {
                 result = std::stod(env_simple_freq_max);
             }
             catch (std::invalid_argument) {

             }
        }
        if (isnan(result)) {
            std::ifstream freq_file(m_cpu_freq_max_path);
            if (freq_file.is_open()) {
                std::string line;
                getline(freq_file, line);
                try {
                    result = 1e4 * std::stod(line);
                }
                catch (std::invalid_argument) {

                }
            }
        }
        if (isnan(result)) {
            try {
                result = cpu_freq_sticker() + m_freq_step;
            }
            catch (Exception) {

            }
        }
        if (isnan(result)) {
            throw Exception("SimpleFreqDecider::cpu_freq_max(): unable to parse maximum frequency",
                            errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        return result;
    }

}

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

#include "TelemetryConfig.hpp"
#include "Exception.hpp"

namespace geopm
{

    TelemetryConfig::TelemetryConfig()
    {

    }

    TelemetryConfig::TelemetryConfig(const TelemetryConfig &other)
        : m_available_signal(other.m_available_signal)
        , m_enabled_signal(other.m_enabled_signal)
        , m_control_bound(other.m_control_bound)
    {

    }

    TelemetryConfig::~TelemetryConfig()
    {

    }

    void TelemetryConfig::available(const std::map<int, std::set<std::string> > &available)
    {
        m_available_signal = available;
    }

    void TelemetryConfig::available(int ctr_domain, std::set<std::string> &available) const
    {
        auto it = m_available_signal.find(ctr_domain);
        if (it == m_available_signal.end()) {
        }
        available = (*it).second;
    }

    void TelemetryConfig::enabled(int ctr_domain, std::set<std::string> &enabled) const
    {
        auto it = m_enabled_signal.find(ctr_domain);
        if (it == m_enabled_signal.end()) {
        }
        enabled = (*it).second;;
    }

    void TelemetryConfig::enable(int ctr_domain, std::set<std::string> &enable)
    {
        auto entry = m_enabled_signal.find(ctr_domain);
        if (entry != m_enabled_signal.end()) {
            (*entry).second.insert(enable.begin(), enable.end());
        }
        else {
            m_enabled_signal.insert(std::pair<int, std::set<std::string> >(ctr_domain, enable);
        }
    }

    void TelemetryConfig::enable(int ctr_domain, std::string &enable)
    {
        auto entry = m_enabled_signal.find(ctr_domain);
        if (entry != m_enabled_signal.end()) {
            (*entry).second.insert(enable);
        }
        else {
            m_enabled_signal.insert(std::pair<int, std::set<std::string> >(ctr_domain, {enable});
        }
    }

    void TelemetryConfig::domain_cpu_map(int domain, std::map<int, std::vector<int> > &domain_map) const
    {
        auto map = m_domain_map.find(domain);
        if (map == m_domain_map.end()) {
            throw Exception("TelemetryConfig::domain_cpu_map(): unknown domain: " +
                            std::to_string(domain),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        domain_map = (*map).second;
    }

    void TelemetryConfig::domain_cpu_map(const std::map<int, std::vector<int, std::set<int> > > &domain_map)
    {
        m_domain_map = domain_map;
    }

    void TelemetryConfig::bounds(const std::map<int, std::pair<double, double> > &bound)
    {
        m_control_bound = bound;
    }

    void TelemetryConfig::bounds(int control_type, double &lower, double &upper) const
    {
        auto bounds = m_control_bound.find(control_type);
        if (bounds == m_control_bound.end()) {
            throw Exception("TelemetryConfig::bounds(): unknown control type: " +
                            std::to_string(control_type),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        lower = (*bounds).second.first;
        upper = (*bounds).second.second;
    }

    void TelemetryConfig::supported_counter_domain(const std::set<int> counter_domain)
    {
        m_counter_domain = counter_domain;
    }

    void TelemetryConfig::supported_control_domain(const std::set<int> control_domain)
    {
        m_control_domain = control_domain;
    }

    bool TelemetryConfig::is_counter_domain_supported(int counter_domain) const
    {
        bool rval= false;
        for (auto it = m_counter_domain.begin(); it != m_counter_domain.end(); ++it) {
            if ((*it) == counter_domain) {
                rval = true;
                break;
            }
        }
        return rval;
    }

    bool TelemetryConfig::is_control_domain_supported(int control_domain) const
    {
        bool rval= false;
        for (auto it = m_control_domain.begin(); it != m_control_domain.end(); ++it) {
            if ((*it) == control_domain) {
                rval = true;
                break;
            }
        }
        return rval;
    }
}

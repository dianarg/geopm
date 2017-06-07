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

#ifndef TELEMETRYCONFIG_HPP_INCLUDE
#define TELEMETRYCONFIG_HPP_INCLUDE

#include <set>
#include <map>
#include <string>

namespace geopm {
    class TelemetryConfig {
        public:
            TelemetryConfig(std::vector<int> fan_out);
            TelemetryConfig(const TelemetryConfig &other);
            virtual ~TelemetryConfig();
            void set_provided(int signal_domain, const std::set<std::string> &available);
            void get_provided(int signal_domain, std::set<std::string> &available) const;
            bool is_provided(int signal_domain, const std::string &available) const;
            void set_required(int signal_domain, const std::set<std::string> &enable);
            void set_required(int signal_domain, const std::string &enable);
            void get_required(int signal_domain, std::set<std::string> &enabled) const;
            bool is_required(int signal_domain, const std::string &enabled) const;
            void set_domain_cpu_map(int domain, const std::vector<std::set<int> > &domain_map);
            void get_domain_cpu_map(int domain, std::vector<std::set<int> > &domain_map) const;
            int num_domain_entry(int domain);
            void set_bounds(int signal_domain, double lower, double upper);
            void get_bounds(int level, int ctl_domain, double &lower, double &upper) const;
            void supported_domain(const std::set<int> domain);
            bool is_supported_domain(int domain) const;
            void set_aggregate(const std::vector<std::string> &agg);
            void get_aggregate(std::vector<std::string> &agg) const;
        private:
            int num_children(int level);
            std::map<int, std::set<std::string> > m_available_signal;
            std::map<int, std::set<std::string> > m_enabled_signal;
            std::map<int, std::set<std::string> > m_aggregate_signal;
            std::map<int, std::pair<double, double> > m_control_bound;
            std::map<int, std::vector<std::set<int> > > m_domain_map;
            std::set<int> m_supported_domain;
            std::vector<int> m_fan_out;
    };
}
#endif

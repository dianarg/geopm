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
#include <signal.h>
#include <map>
#include <sstream>

#include "PlatformTopo.hpp"
#include "Exception.hpp"

extern "C"
{
    static volatile unsigned g_is_popen_complete = 0;
    static struct sigaction g_popen_complete_signal_action;

    static void geopm_popen_complete(int signum)
    {
        if (signum == SIGCHLD) {
            g_is_popen_complete = 1;
        }
    }
}


namespace geopm
{
    class PlatformTopo : public IPlatformTopo
    {
        public:
            PlatformTopo();
            virtual ~PlatformTopo();
            int num_domain(int domain_type) const;
            void domain_cpus(int domain_type,
                             int domain_idx,
                             std::set<int> &cpu_idx) const;
            int domain_idx(int domain_type,
                           int cpu_idx) const;
            int define_cpu_group(const std::vector<int> &cpu_domain_idx);
        protected:
            void lscpu(std::map<std::string, std::string> &lscpu_map);
            void parse_lscpu(const std::map<std::string, std::string> &lscpu_map,
                             int &num_package,
                             int &core_per_package,
                             int &thread_per_core);
            void parse_lscpu_numa(std::map<std::string, std::string> lscpu_map,
                                  std::vector<std::set<int> > &numa_map);
            virtual FILE *open_cmd(std::string cmd);
            virtual void close_cmd(FILE *);

            int m_num_package;
            int m_core_per_package;
            int m_thread_per_core;
            std::vector<std::set<int> > m_numa_map;
    };

    IPlatformTopo &platform_topo(void)
    {
        static PlatformTopo instance;
        return instance;
    }

    PlatformTopo::PlatformTopo()
    {
        std::map<std::string, std::string> lscpu_map;
        lscpu(lscpu_map);
        parse_lscpu(lscpu_map, m_num_package, m_core_per_package, m_thread_per_core);
        parse_lscpu_numa(lscpu_map, m_numa_map);
    }

    void PlatformTopo::parse_lscpu(const std::map<std::string, std::string> &lscpu_map,
                                   int &num_package,
                                   int &core_per_package,
                                   int &thread_per_core)
    {
        std::string keys[5] = {"CPU(s)",
                                "Thread(s) per core",
                                "Core(s) per socket",
                                "Socket(s)",
                                "NUMA node(s)"};
        int values[5] = {};
        int num_values = sizeof(values) / sizeof(values[0]);

        for (int i = 0; i < num_values; ++i) {
            auto it = lscpu_map.find(keys[i]);
            if (it == lscpu_map.end()) {
                throw Exception("PlatformTopo: Error parsing lscpu output, key not found",
                                GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
            }
            values[i] = atoi(it->second.c_str());
            if (!values[i]) {
                throw Exception("PlatformTopo: Error parsing lscpu output, value not converted",
                                GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
            }
        }
        num_package = values[3];
        int num_core = values[2] * m_num_package;
        core_per_package = m_num_package / num_core;
        thread_per_core = values[1];
        if (num_package * core_per_package * thread_per_core != values[0]) {
            throw Exception("PlatformTopo: Error parsing lscpu output, inconsistant values or come CPUs are not online",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
    }
    void PlatformTopo::parse_lscpu_numa(std::map<std::string, std::string> lscpu_map,
                                        std::vector<std::set<int> > &numa_map)
    {
        bool is_node_found = true;
        for (int node_idx = 0; is_node_found; ++node_idx) {
            std::ostringstream numa_key;
            numa_key << "NUMA node" << node_idx << " CPU(s)";
            auto lscpu_it = lscpu_map.find(numa_key.str());
            if (lscpu_it == lscpu_map.end()) {
                is_node_found = false;
            }
            else if (lscpu_it->second.size()) {
                std::string cpu_list = lscpu_it->second;
                numa_map.push_back({});
                auto cpu_set_it = numa_map.end() - 1;
                bool is_comma_found = true;
                while (is_comma_found) {
                    cpu_set_it->insert(atoi(cpu_list.c_str()));
                    size_t comma_idx = cpu_list.find(',');
                    if (comma_idx != std::string::npos) {
                        cpu_list = cpu_list.substr(comma_idx + 1);
                    }
                    else {
                        is_comma_found = false;
                    }
                }
            }
        }
    }

    FILE *PlatformTopo::open_cmd(std::string cmd)
    {
        FILE *result = nullptr;
        struct sigaction save_action;
        g_popen_complete_signal_action.sa_handler = geopm_popen_complete;
        sigemptyset(&g_popen_complete_signal_action.sa_mask);
        g_popen_complete_signal_action.sa_flags = 0;
        int err = sigaction(SIGCHLD, &g_popen_complete_signal_action, &save_action);
        if (!err) {
            result = popen(cmd.c_str(), "r");
            while (!g_is_popen_complete) {

            }
            g_is_popen_complete = 0;
            sigaction(SIGCHLD, &save_action, NULL);
        }
        return result;
    }
    void PlatformTopo::close_cmd(FILE *fid)
    {
        pclose(fid);
    }

    void PlatformTopo::lscpu(std::map<std::string, std::string> &lscpu_map)
    {
        std::string result;

        FILE *fid = open_cmd("lscpu");
        if (fid) {
            char key[128] = "";
            char value[128] = "";
            key[127] = '\0';
            value[127] = '\0';
            size_t num_read = fscanf(fid, "%127s: %127s\n", key, value);
            if (num_read == 2) {
                lscpu_map.emplace(key, value);
            }
        }
    }
}

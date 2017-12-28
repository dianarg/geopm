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

#include "PlatformTopo.hpp"

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
            int num_domain(uint64_t domain_type) const;
            void domain_cpus(uint64_t domain_type, int domain_idx, std::set<int> &cpu_idx) const;
            int domain_idx(uint64_t domain_type, int cpu_idx) const;
            uint64_t define_cpu_group(const std::vector<int> &cpu_domain_idx);
        protected:
            void lscpu(std::map<std::string, std::string> &cpu_map);
            virtual FILE *open_cmd(std::string cmd);
            virtual void close_cmd(FILE *);

            struct sigaction m_save_action;
            int m_num_package;
            int m_core_per_package;
            int m_thread_per_core;
    }

    IPlatformTopo &platform_topo(void)
    {
        static PlatformTopo instance;
        return instance;
    }

    PlatformTopo::PlatformTopo()
    {
        std::map<std::string, std::string> lscpu_map;
        std::string ikeys[5] = {"CPU(s)",
                                "Thread(s) per core",
                                "Core(s) per socket",
                                "Socket(s)",
                                "NUMA node(s)"}
        int ivalues[5] = {};

        std::string vkeys[2] = {"On-line CPU(s) list",
                                "NUMA node0 CPU(s)"}
        std::vector<int> vvalues[2] = {};

        lscpu(lscpu_map);

        for (int i = 0; i < 5; ++i) {
            auto it = lscpu_map.find(keys[i]);
            if (it == lscpu_map.end()) {
                throw Exception("PlatformTopo: Error parsing lscpu output, key not found",
                                GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
            }
            ivalues[i] = atoi(it->second);
            if (!value[i]) {
                throw Exception("PlatformTopo: Error parsing lscpu output, value not converted",
                                GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
            }
        }
        m_num_package = value[3];
        int num_core = value[2] * value[3];
        m_num_core_per_package = m_num_package / num_core;
        m_num_thread_per_core = value[1];
        if (m_num_package * m_num_core_per_package * m_num_thread_per_core != value[0]) {
            throw Exception("PlatformTopo: Error parsing lscpu output, inconsistant values",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
    }

    FILE *PlatformTopo::open_cmd(std::string cmd)
    {
        FILE *result = nullptr;
        g_popen_complete_signal_action.sa_handler = geopm_popen_complete;
        sigemptyset(&g_popen_complete_signal_action.sa_mask);
        g_popen_complete_signal_action.sa_flags = 0;
        int err = sigaction(SIGCHLD, &g_popen_complete_signal_action, &m_save_action);
        if (!err) {
            result = popen(cmd.c_str(), "r");
            while (!g_is_popen_complete) {

            }
            g_is_popen_complete = 0;
            sigaction(SIGCHLD, &m_save_action, NULL);
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
            size_t num_read = fscanf("%127s: %127s\n", key, value);
            if (num_read == 2) {
                lscpu_map.emplace(key, value);
            }
        }
    }
}

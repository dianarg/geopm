/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
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

#ifndef BUSYWAITIOGROUP_HPP_INCLUDE
#define BUSYWAITIOGROUP_HPP_INCLUDE

#include "IOGroup.hpp"

namespace geopm
{
    class PlatformTopo;

    class BusyWaitIOGroup : public IOGroup
    {
        public:
            BusyWaitIOGroup();
            virtual ~BusyWaitIOGroup() = default;
            std::set<std::string> signal_names(void) const override;
            std::set<std::string> control_names(void) const override;
            bool is_valid_signal(const std::string &signal_name) const override;
            bool is_valid_control(const std::string &control_name) const override;
            int signal_domain_type(const std::string &signal_name) const override;
            int control_domain_type(const std::string &control_name) const override;
            int push_signal(const std::string &signal_name,
                            int domain_type,
                            int domain_idx) override;
            int push_control(const std::string &control_name,
                             int domain_type,
                             int domain_idx) override;
            void read_batch(void) override;
            void write_batch(void) override;
            double sample(int sample_idx) override;
            void adjust(int control_idx,
                        double setting) override;
            double read_signal(const std::string &signal_name,
                               int domain_type,
                               int domain_idx) override;
            void write_control(const std::string &control_name,
                               int domain_type,
                               int domain_idx,
                               double setting) override;
            void save_control(void) override;
            void restore_control(void) override;
            std::function<double(const std::vector<double> &)> agg_function(const std::string &signal_name) const override;
            std::function<std::string(double)> format_function(const std::string &signal_name) const override;
            std::string signal_description(const std::string &signal_name) const override;
            std::string control_description(const std::string &control_name) const override;
            /// @brief Regex matching function for checking if a
            /// function name matches regex values marked as busy
            /// wait.
            static bool regex_match(const std::string &regex,
                                    const std::string &query);
            /// @brief Returns a vector that stores the process id of
            /// the MPI process running on each linux logical CPU.
            std::vector<int> get_cpu_pid_map(void) const;
            /// @brief Returns the logical equivalent of KSTK_EIP()
            /// for a process ID provided.  This may be derived from
            /// the /proc/<proc_id>/stat field (30) labeled kstkeip in
            /// proc(5) man page.
            size_t get_eip(int proc_id) const;
            /// @brief Returns the path to the mpi library shared object file.
            std::string get_libmpi_path(void) const;
            /// @brief Returns the offset from /proc/<proc_id>/smaps
            /// where the mpi library is mapped in a vector indexed
            /// over the linux logical CPU's where MPI processes are
            /// running.
            std::vector<size_t> get_libmpi_offset(void) const;
            /// @brief Returns true of symbol name matches regex
            /// values for busy wait functions.
            bool is_busy_symbol(const std::string &symbol_name) const;
            /// @brief Returns true if the eip for the process running
            /// on the indexed cpu is within a busy wait function.
            bool is_busy_wait(int cpu_idx, size_t eip) const;
            static std::string plugin_name(void);
            static std::unique_ptr<IOGroup> make_plugin(void);
        private:
            std::map<size_t, bool> create_busy_wait(void);
            static const std::string M_PLUGIN_NAME;
            const PlatformTopo &m_platform_topo;
            const std::vector<std::string> m_busy_names;
            const int m_num_cpu;
            const std::vector<int> m_cpu_pid_map;
            const std::vector<uint64_t> m_cpu_libmpi_offset_map;
            const std::set<std::string> m_signal_names;
            const std::string m_description;
            // Map contains the start and end address ranges of the
            // busy wait function matches.  If the address is a begin
            // address then the bool is true, if it is an end address
            // the bool is false.
            std::map<size_t, bool> m_is_addr_busy_wait;
    };
}

#endif

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

#include "BusyWaitIOGroup.hpp"
#include "Helper.hpp"
#include "Exception.hpp"
#include "PlatformTopo.hpp"
#include "ELF.hpp"
#include "Agg.hpp"

namespace geopm
{
    const std::string BusyWaitIOGroup::M_PLUGIN_NAME = "BUSY_WAIT";

    BusyWaitIOGroup::BusyWaitIOGroup()
        : m_platform_topo(platform_topo())
        , m_busy_names {
                "\bMPID_nem_.*_recv",
                "\bMPID.*poll.*",
                "\bP?MPIDI_CH3I_Progress",
                "\bP?I_MPI_PW_Sched_yield",
                "\bMPID_nem_lmt_.*_progress",
                "\bMPIDI_Progress_test",
                "\bMPIDI_NM_progress",
                "\bMPIDI_SHM_progress",
                "\bfi_cq_read",
                "\bMPID_Progress_wait",
            }
        , m_num_cpu(m_platform_topo.num_domain(GEOPM_DOMAIN_CPU))
        , m_cpu_pid_map(get_cpu_pid_map())
        , m_cpu_libmpi_offset_map(get_libmpi_offset())
        , m_signal_names {
                "IS_BUSY_WAIT",
                "BUSY_WAIT::IS_BUSY_WAIT"
            }
        , m_description("True (1) if CPU is executing a busy wait region of MPI, false otherwise (0)")
        , m_is_addr_busy_wait(create_busy_wait())
    {

    }

    std::string BusyWaitIOGroup::get_libmpi_path(void) const
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__),
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    std::vector<size_t> BusyWaitIOGroup::get_libmpi_offset(void) const
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__),
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    std::map<size_t, bool> BusyWaitIOGroup::create_busy_wait(void)
    {
        std::string exe_path = get_libmpi_path();
        std::map<size_t, bool> result;
        std::map<size_t, std::string> symbol_map = elf_symbol_map(exe_path);
        for (auto symbol_it = symbol_map.cbegin();
             symbol_it != symbol_map.cend();
             ++symbol_it) {
            if (is_busy_symbol(symbol_it->second)) {
                result[symbol_it->first] = true;
                auto next_symbol_it = symbol_it;
                ++next_symbol_it;
                if (next_symbol_it != symbol_map.end()) {
                    result[next_symbol_it->first] = false;
                }
            }
        }
        return result;
    }

    bool BusyWaitIOGroup::is_busy_symbol(const std::string &symbol_name) const
    {
        bool result = false;
        for (const auto &name_it : m_busy_names) {
            if (regex_match(name_it, symbol_name)) {
                result = true;
                break;
            }
        }
        return result;
    }

    std::vector<int> BusyWaitIOGroup::get_cpu_pid_map(void) const
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__),
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    bool BusyWaitIOGroup::regex_match(const std::string &regex,
                                      const std::string &query)
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__),
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    size_t BusyWaitIOGroup::get_eip(int proc_id) const
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__),
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    std::set<std::string> BusyWaitIOGroup::signal_names(void) const
    {
        return m_signal_names;
    }

    std::set<std::string> BusyWaitIOGroup::control_names(void) const
    {
        return {};
    }

    bool BusyWaitIOGroup::is_valid_signal(const std::string &signal_name) const
    {
        return m_signal_names.find(signal_name) != m_signal_names.end();
    }

    bool BusyWaitIOGroup::is_valid_control(const std::string &control_name) const
    {
        return false;
    }

    int BusyWaitIOGroup::signal_domain_type(const std::string &signal_name) const
    {
        int result = GEOPM_DOMAIN_INVALID;
        if (is_valid_signal(signal_name)) {
            result = GEOPM_DOMAIN_CPU;
        }
        return result;
    }

    int BusyWaitIOGroup::control_domain_type(const std::string &control_name) const
    {
        return GEOPM_DOMAIN_INVALID;
    }

    int BusyWaitIOGroup::push_signal(const std::string &signal_name,
                                     int domain_type,
                                     int domain_idx)
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__),
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    int BusyWaitIOGroup::push_control(const std::string &control_name,
                                      int domain_type,
                                      int domain_idx)
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__) +
                        ": no controls supported",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    void BusyWaitIOGroup::read_batch(void)
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__),
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    void BusyWaitIOGroup::write_batch(void)
    {

    }

    double BusyWaitIOGroup::sample(int sample_idx)
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__),
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    void BusyWaitIOGroup::adjust(int control_idx,
                                 double setting)
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__) +
                        ": no controls supported",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    double BusyWaitIOGroup::read_signal(const std::string &signal_name,
                                        int domain_type,
                                        int domain_idx)
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("BusyWaitIOGroup::" + std::string(__func__) +
                            ": invalid signal name: " +
                            signal_name,
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (GEOPM_DOMAIN_CPU != domain_type) {
            throw Exception("BusyWaitIOGroup::" + std::string(__func__) +
                            ": invalid domain type: " +
                            PlatformTopo::domain_type_to_name(domain_type),
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(domain_type)) {
            throw Exception("BusyWaitIOGroup::read_signal(): domain_idx out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        double result = 0.0;
        int proc_id = m_cpu_pid_map[domain_idx];
        if (proc_id > 0) {
            // The CPU is running an MPI process we are tracking
            size_t eip = get_eip(proc_id);
            result = is_busy_wait(domain_idx, eip) ? 1.0 : 0.0;
        }
        return result;
    }

    bool BusyWaitIOGroup::is_busy_wait(int cpu_idx, size_t eip) const
    {
        bool result = false;
        size_t offset = m_cpu_libmpi_offset_map[cpu_idx];
        if (eip > offset) {
            auto addr_it = m_is_addr_busy_wait.upper_bound(eip - offset);
            if (!addr_it->second) {
                result = true;
            }
        }
        return result;
    }

    void BusyWaitIOGroup::write_control(const std::string &control_name,
                                        int domain_type,
                                        int domain_idx,
                                        double setting)
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__) +
                        ": no controls supported",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    void BusyWaitIOGroup::save_control(void)
    {

    }

    void BusyWaitIOGroup::restore_control(void)
    {

    }

    std::function<double(const std::vector<double> &)> BusyWaitIOGroup::agg_function(const std::string &signal_name) const
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("CpuinfoIOGroup::agg_function(): unknown how to aggregate \"" + signal_name + "\"",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return Agg::expect_same;
    }

    std::function<std::string(double)> BusyWaitIOGroup::format_function(const std::string &signal_name) const
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("CpuinfoIOGroup::agg_function(): signal_name \"" +
                            signal_name + "\" unknown aggregation function",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return string_format_integer;
    }

    std::string BusyWaitIOGroup::signal_description(const std::string &signal_name) const
    {
        if (is_valid_signal(signal_name)) {
            return m_description;
        }
        else {
            throw Exception("BusyWaitIOGroup::signal_description(): signal_name \"" +
                            signal_name + "\" not valid for BusyWaitIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    std::string BusyWaitIOGroup::control_description(const std::string &control_name) const
    {
        throw Exception("BusyWaitIOGroup::" + std::string(__func__) +
                        ": no controls supported",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    std::string BusyWaitIOGroup::plugin_name(void)
    {
        return M_PLUGIN_NAME;
    }

    std::unique_ptr<IOGroup> BusyWaitIOGroup::make_plugin(void)
    {
        return geopm::make_unique<BusyWaitIOGroup>();
    }

}

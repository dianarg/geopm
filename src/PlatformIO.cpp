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

#include <cpuid.h>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>

#include "geopm_sched.h"
#include "geopm_message.h"
#include "geopm_hash.h"
#include "geopm.h"
#include "PlatformIO.hpp"
#include "PlatformIOInternal.hpp"
#include "PlatformTopo.hpp"
#include "MSRIOGroup.hpp"
#include "TimeIOGroup.hpp"
#include "Exception.hpp"
#include "Helper.hpp"
#include "Agg.hpp"

#include "config.h"

namespace geopm
{
    IPlatformIO &platform_io(void)
    {
        static PlatformIO instance;
        return instance;
    }

    PlatformIO::PlatformIO()
        : PlatformIO({}, platform_topo())
    {

    }

    PlatformIO::PlatformIO(std::list<std::shared_ptr<IOGroup> > iogroup_list,
                           IPlatformTopo &topo)
        : m_is_active(false)
        , m_platform_topo(topo)
        , m_iogroup_list(iogroup_list)
        , m_do_restore(false)
    {
        if (m_iogroup_list.size() == 0) {
            for (const auto &it : iogroup_factory().plugin_names()) {
                try {
                    register_iogroup(iogroup_factory().make_plugin(it));
                }
                catch (const geopm::Exception &ex) {
#ifdef GEOPM_DEBUG
                    std::cerr << "<geopm> Warning: failed to load " << it << " IOGroup.  "
                              << "GEOPM may not work properly unless an alternate "
                              << "IOGroup plugin is loaded to provide signals/controls "
                              << "required by the Controller and Agent."
                              << std::endl;
                    std::cerr << "The error was: " << ex.what() << std::endl;
#endif
                }
            }
        }
    }

    SignalGroup::SignalGroup()
    {
        m_signal_info["POWER_PACKAGE"] = {{"REGION_ID#", "TIME", "ENERGY_PACKAGE"}, "ENERGY_PACKAGE"};
        m_signal_info["POWER_DRAM"] = {{"REGION_ID#", "TIME", "ENERGY_DRAM"}, "ENERGY_DRAM"};
    }

    void PlatformIO::register_iogroup(std::shared_ptr<IOGroup> iogroup)
    {
        if (m_do_restore) {
            throw Exception("PlatformIO::register_iogroup(): "
                            "IOGroup cannot be registered after a call to save_control()",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        m_iogroup_list.push_back(iogroup);
    }

    std::set<std::string> PlatformIO::signal_names(void) const
    {
        std::set<std::string> result;
        for (const auto &io_group : m_iogroup_list) {
            auto names = io_group->signal_names();
            result.insert(names.begin(), names.end());
        }
        std::set<std::string> compound_signals = m_signal_group.signal_names(result);
        result.insert(compound_signals.begin(), compound_signals.end());
        return result;
    }

    std::set<std::string> SignalGroup::signal_names(const std::set<std::string> &available_signals) const
    {
        std::set<std::string> result;
        // loop through provided signals and check that signals they depend on are available
        for (const auto &sig : m_signal_info) {
            if (is_valid_signal(sig.first, available_signals)) {
                result.insert(sig.first);
            }
        }
        return result;
    }

    bool SignalGroup::is_valid_signal(const std::string &signal_name, const std::set<std::string> &available_signals) const
    {
        // signal is only valid if all dependencies are available
        bool valid = false;
        auto it = m_signal_info.find(signal_name);
        if (it != m_signal_info.end()) {
            valid = true;
            for (const auto &rs : it->second.required_signals) {
                if (available_signals.find(rs) == available_signals.end()) {
                    valid = false;
                }
            }
            if (available_signals.find(it->second.domain_signal) == available_signals.end()) {
                valid = false;
            }
        }
        std::cout << "is_valid " << signal_name << " " << valid << std::endl;
        return valid;
    }

    std::vector<std::string> SignalGroup::required_signals(const std::string &signal_name) const
    {
        // todo: better error message
        return m_signal_info.at(signal_name).required_signals;
    }

    std::set<std::string> PlatformIO::control_names(void) const
    {
        std::set<std::string> result;
        for (const auto &io_group : m_iogroup_list) {
            auto names = io_group->control_names();
            result.insert(names.begin(), names.end());
        }
        return result;
    }

    int PlatformIO::signal_domain_type(const std::string &signal_name) const
    {
        int result = PlatformTopo::M_DOMAIN_INVALID;
        bool is_found = false;
        for (auto it = m_iogroup_list.rbegin();
             !is_found && it != m_iogroup_list.rend();
             ++it) {
            if ((*it)->is_valid_signal(signal_name)) {
                result = (*it)->signal_domain_type(signal_name);
                is_found = true;
            }
        }
        if (!is_found && m_signal_group.is_valid_signal(signal_name, signal_names())) {
            result = signal_domain_type(m_signal_group.domain_signal(signal_name));
            is_found = true;
        }
        if (!is_found) {
            throw Exception("PlatformIO::signal_domain_type(): signal name \"" +
                            signal_name + "\" not found",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return result;
    }

    std::string SignalGroup::domain_signal(const std::string &signal_name) const
    {
        std::string result;
        if (m_signal_info.find(signal_name) != m_signal_info.end()) {
            result = m_signal_info.at(signal_name).domain_signal;
        }
        std::cout << result << std::endl;
        return result;
    }

    int PlatformIO::control_domain_type(const std::string &control_name) const
    {
        int result = PlatformTopo::M_DOMAIN_INVALID;
        bool is_found = false;
        for (auto it = m_iogroup_list.rbegin();
             !is_found && it != m_iogroup_list.rend();
             ++it) {
            if ((*it)->is_valid_control(control_name)) {
                result = (*it)->control_domain_type(control_name);
                is_found = true;
            }
        }
        if (!is_found) {
            throw Exception("PlatformIO::control_domain_type(): control name \"" +
                            control_name + "\" not found",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return result;
    }

    int PlatformIO::push_signal(const std::string &signal_name,
                                int domain_type,
                                int domain_idx)
    {
        if (m_is_active) {
            throw Exception("PlatformIO::push_signal(): pushing signals after read_batch() or adjust().",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        int result = -1;
        auto sig_tup = std::make_tuple(signal_name, domain_type, domain_idx);
        auto sig_tup_it = m_existing_signal.find(sig_tup);
        if (sig_tup_it != m_existing_signal.end()) {
            result = sig_tup_it->second;
        }
        if (result == -1) {
            for (auto it = m_iogroup_list.rbegin();
                 result == -1 && it != m_iogroup_list.rend();
                 ++it) {
                if ((*it)->is_valid_signal(signal_name) &&
                    (*it)->signal_domain_type(signal_name) == domain_type) {
                    int group_signal_idx = (*it)->push_signal(signal_name, domain_type, domain_idx);
                    result = m_active_signal.size();
                    m_existing_signal[sig_tup] = result;
                    m_active_signal.emplace_back((*it).get(), group_signal_idx);
                }
            }
        }
        if (result == -1) {
            result = push_signal_power(signal_name, domain_type, domain_idx);
        }
        if (result == -1) {
            result = push_signal_convert_domain(signal_name, domain_type, domain_idx);
        }
        if (result == -1) {
            throw Exception("PlatformIO::push_signal(): no support for signal name \"" +
                            signal_name + "\" and domain type \"" +
                            std::to_string(domain_type) + "\"",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        else {
            m_existing_signal[sig_tup] = result;
        }
        return result;
    }

    // todo: rename
    int PlatformIO::push_signal_power(const std::string &signal_name,
                                      int domain_type,
                                      int domain_idx)
    {
        int result = -1;
        if (m_signal_group.is_valid_signal(signal_name, signal_names())) {
            std::vector<int> req_idx;
            for (const auto &sig : m_signal_group.required_signals(signal_name)) {
                req_idx.push_back(push_signal(sig, domain_type, domain_idx));
            }
            result = m_active_signal.size();

            // TODO: this assumes all SignalGroup signals use per region derivative;
            // need a function that retures a unique_ptr<CombinedSignal>
            register_combined_signal(result, req_idx,
                                     std::unique_ptr<CombinedSignal>(new PerRegionDerivativeCombinedSignal));

            m_active_signal.emplace_back(nullptr, result);
        }
        return result;
    }

    int PlatformIO::push_signal_convert_domain(const std::string &signal_name,
                                               int domain_type,
                                               int domain_idx)
    {
        int result = -1;
        int base_domain_type = signal_domain_type(signal_name);
        if (m_platform_topo.is_domain_within(base_domain_type, domain_type)) {
            std::set<int> cpus;
            m_platform_topo.domain_cpus(domain_type, domain_idx, cpus);
            std::set<int> base_domain_idx;
            for (auto it : cpus) {
                base_domain_idx.insert(m_platform_topo.domain_idx(base_domain_type, it));
            }
            std::vector<int> signal_idx;
            for (auto it : base_domain_idx) {
                signal_idx.push_back(push_signal(signal_name, base_domain_type, it));
            }
            result = push_combined_signal(signal_name, domain_type, domain_idx, signal_idx);
        }
        return result;
    }

    int PlatformIO::push_combined_signal(const std::string &signal_name,
                                         int domain_type,
                                         int domain_idx,
                                         const std::vector<int> &sub_signal_idx)
    {
        int result = m_active_signal.size();
        std::unique_ptr<CombinedSignal> combiner = geopm::make_unique<CombinedSignal>(agg_function(signal_name));
        register_combined_signal(result, sub_signal_idx, std::move(combiner));
        m_active_signal.emplace_back(nullptr, result);
        return result;
    }


    void PlatformIO::register_combined_signal(int signal_idx,
                                              std::vector<int> operands,
                                              std::unique_ptr<CombinedSignal> signal)
    {
        auto tmp = std::make_pair(operands, std::move(signal));
        m_combined_signal[signal_idx] = std::move(tmp);
    }

    int PlatformIO::push_control(const std::string &control_name,
                                 int domain_type,
                                 int domain_idx)
    {
        if (m_is_active) {
            throw Exception("PlatformIO::push_control(): pushing controls after read_batch() or adjust().",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        int result = -1;
        auto ctl_tup = std::make_tuple(control_name, domain_type, domain_idx);
        auto ctl_tup_it = m_existing_control.find(ctl_tup);
        if (ctl_tup_it != m_existing_control.end()) {
            result = ctl_tup_it->second;
        }
        if (result == -1) {
            bool is_found = false;
            for (auto it = m_iogroup_list.rbegin();
                 !is_found && it != m_iogroup_list.rend();
                 ++it) {
                if ((*it)->is_valid_control(control_name)) {
                    int group_control_idx = (*it)->push_control(control_name, domain_type, domain_idx);
                    result = m_active_control.size();
                    m_existing_control[ctl_tup] = result;
                    m_active_control.emplace_back((*it).get(), group_control_idx);
                    is_found = true;
                }
            }
        }
        if (result == -1) {
            throw Exception("PlatformIO::push_control(): control name \"" +
                            control_name + "\" not found",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return result;
    }

    int PlatformIO::num_signal(void) const
    {
        return m_active_signal.size();
    }

    int PlatformIO::num_control(void) const
    {
        return m_active_control.size();
    }

    double PlatformIO::sample(int signal_idx)
    {
        double result = NAN;
        if (signal_idx < 0 || signal_idx >= num_signal()) {
            throw Exception("PlatformIO::sample(): signal_idx out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (!m_is_active) {
            throw Exception("PlatformIO::sample(): read_batch() not called prior to call to sample()",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        auto &group_idx_pair = m_active_signal[signal_idx];
        if (group_idx_pair.first) {
            result = group_idx_pair.first->sample(group_idx_pair.second);
        }
        else {
            result = sample_combined(group_idx_pair.second);
        }
        return result;
    }

    double PlatformIO::sample_combined(int signal_idx)
    {
        double result = NAN;
        auto &op_func_pair = m_combined_signal.at(signal_idx);
        std::vector<int> &operand_idx = op_func_pair.first;
        auto &signal = op_func_pair.second;
        std::vector<double> operands(operand_idx.size());
        for (size_t ii = 0; ii < operands.size(); ++ii) {
            operands[ii] = sample(operand_idx[ii]);
        }
        result = signal->sample(operands);
        return result;
    }

    void PlatformIO::adjust(int control_idx,
                            double setting)
    {
        if (control_idx < 0 || control_idx >= num_control()) {
            throw Exception("PlatformIO::adjust(): control_idx out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (std::isnan(setting)) {
            throw Exception("PlatformIO::adjust(): setting is NAN",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        auto &group_idx_pair = m_active_control[control_idx];
        group_idx_pair.first->adjust(group_idx_pair.second, setting);
        m_is_active = true;
    }

    void PlatformIO::read_batch(void)
    {
        for (auto &it : m_iogroup_list) {
            it->read_batch();
        }
        m_is_active = true;
    }

    void PlatformIO::write_batch(void)
    {
        for (auto &it : m_iogroup_list) {
            it->write_batch();
        }
    }

    double PlatformIO::read_signal(const std::string &signal_name,
                                   int domain_type,
                                   int domain_idx)
    {
        double result = 0.0;
        bool is_found = false;
        for (auto it = m_iogroup_list.rbegin();
             !is_found && it != m_iogroup_list.rend();
             ++it) {
            if ((*it)->is_valid_signal(signal_name)) {
                result = (*it)->read_signal(signal_name, domain_type, domain_idx);
                is_found = true;
            }
        }
        if (!is_found) {
            throw Exception("PlatformIO::read_signal(): signal name \"" + signal_name + "\" not found",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return result;
    }

    void PlatformIO::write_control(const std::string &control_name,
                                   int domain_type,
                                   int domain_idx,
                                   double setting)
    {
        bool is_found = false;
        for (auto it = m_iogroup_list.rbegin();
             !is_found && it != m_iogroup_list.rend();
             ++it) {
            if ((*it)->is_valid_control(control_name)) {
                (*it)->write_control(control_name, domain_type, domain_idx, setting);
                is_found = true;
            }
        }
        if (!is_found) {
            throw Exception("PlatformIO::write_control(): control name \"" + control_name + "\" not found",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    void PlatformIO::save_control(void)
    {
        m_do_restore = true;
        for (auto it = m_iogroup_list.begin();
             it != m_iogroup_list.end();
             ++it) {
            (*it)->save_control();
        }
    }

    void PlatformIO::restore_control(void)
    {
        if (m_do_restore) {
            for (auto it = m_iogroup_list.begin();
                 it != m_iogroup_list.end();
                 ++it) {
                (*it)->restore_control();
            }
        }
    }

    std::function<double(const std::vector<double> &)> PlatformIO::agg_function(const std::string &signal_name) const
    {
        // Special signals from PlatformIO
        if (m_signal_group.is_valid_signal(signal_name, signal_names())) {
            return m_signal_group.agg_function(signal_name);
        }
        // Find the most recently loaded IOGroup that provides the signal
        auto it = m_iogroup_list.rbegin();
        for (; it != m_iogroup_list.rend(); ++it) {
            if ((*it)->is_valid_signal(signal_name)) {
                break;
            }
        }
        if (it == m_iogroup_list.rend()) {
            throw Exception("PlatformIO::agg_function(): unknown how to aggregate \"" + signal_name + "\"",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return (*it)->agg_function(signal_name);
    }

    std::function<double(const std::vector<double> &)> SignalGroup::agg_function(const std::string &signal_name) const
    {
        static const std::map<std::string, std::function<double(const std::vector<double> &)> > fn_map {
            {"POWER_PACKAGE", Agg::sum},
            {"POWER_DRAM", Agg::sum}
        };
        auto it = fn_map.find(signal_name);
#ifdef GEOPM_DEBUG
        if (it == fn_map.end()) {
            throw Exception("PlatformIO::agg_function(): unknown how to aggregate \"" + signal_name + "\"",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        return it->second;
    }
}

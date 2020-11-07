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

#include "config.h"
#include "SSTIOGroup.hpp"

#include <algorithm>


#include "geopm_debug.hpp"
#include "PlatformTopo.hpp"
#include "Helper.hpp"
#include "Exception.hpp"
#include "Agg.hpp"
#include "MSR.hpp"
#include "MSRFieldSignal.hpp"
#include "SST.hpp"
#include "SSTImp.hpp"
#include "SSTSignal.hpp"

namespace geopm
{
    SSTIOGroup::SSTIOGroup(const PlatformTopo &topo,
                           std::shared_ptr<SSTTransaction> trans)
        : m_is_signal_pushed(false)
        , m_is_batch_read(false)
        , m_valid_signal_name({"ISST::CONFIG_LEVEL"})
        , m_topo(topo)
        , m_trans(trans)
        , m_is_read(false)
    {
        // TODO: need SST::make_shared() function in interface
        if (m_trans == nullptr) {
            m_trans = std::make_shared<SSTTransactionImp>();
        }
    }

    std::set<std::string> SSTIOGroup::signal_names(void) const
    {
        return m_valid_signal_name;
    }

    std::set<std::string> SSTIOGroup::control_names(void) const
    {
        return {};
    }

    bool SSTIOGroup::is_valid_signal(const std::string &signal_name) const
    {
        return true;
        //return m_valid_signal_name.find(signal_name) != m_valid_signal_name.end();
    }

    bool SSTIOGroup::is_valid_control(const std::string &control_name) const
    {
        return false;
    }

    int SSTIOGroup::signal_domain_type(const std::string &signal_name) const
    {
        int result = GEOPM_DOMAIN_INVALID;
        if (is_valid_signal(signal_name)) {
            result = GEOPM_DOMAIN_CPU;
        }
        return result;
    }

    int SSTIOGroup::control_domain_type(const std::string &control_name) const
    {
        return GEOPM_DOMAIN_INVALID;
    }

    int SSTIOGroup::push_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        int result = -1;
        if (signal_name == "ISST::CONFIG_LEVEL") {
            if (domain_type != GEOPM_DOMAIN_PACKAGE) {
                throw Exception("wrong domain type", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
            // TODO: assumes using any CPU in package is fine
            auto cpus = m_topo.domain_nested(GEOPM_DOMAIN_CPU, domain_type, domain_idx);
            int cpu_idx = *(cpus.begin());
            // "ISST::LEVELS_INFO#"
            std::shared_ptr<Signal> levels_info = std::make_shared<SSTSignal>(
                m_trans, cpu_idx, 0x7F, 0x00, 0x00, 0x00);
            std::shared_ptr<Signal> signal = std::make_shared<MSRFieldSignal>(
                levels_info, 16, 23, MSR::M_FUNCTION_SCALE, 1.0);

            // TODO: see linear search in MSRIO::push_signal to check for already pushed
            result = m_signal_pushed.size();
            m_signal_pushed.push_back(signal);
            signal->setup_batch();
        }
        else {
            throw Exception("invalid signal", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        // if (!is_valid_signal(signal_name)) {
        //     throw Exception("SSTIOGroup::push_signal(): signal_name " + signal_name +
        //                     " not valid for SSTIOGroup",
        //                     GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        // }
        // if (domain_type != GEOPM_DOMAIN_CPU) {
        //     throw Exception("SSTIOGroup::push_signal(): signal_name " + signal_name +
        //                     " not defined for domain " + std::to_string(domain_type),
        //                     GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        // }
        // if (m_is_batch_read) {
        //     throw Exception("SSTIOGroup::push_signal(): cannot push signal after call to read_batch().",
        //                     GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        // }
        // m_is_signal_pushed = true;
        // return 0;
        return result;
    }

    int SSTIOGroup::push_control(const std::string &control_name, int domain_type, int domain_idx)
    {
        throw Exception("SSTIOGroup::push_control(): there are no controls supported by the SSTIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    void SSTIOGroup::read_batch(void)
    {
        // if (m_is_signal_pushed) {
        //     m_time_curr = geopm_time_since(&m_time_zero);
        // }
        m_trans->read_batch();
        m_is_read = true;
    }

    void SSTIOGroup::write_batch(void)
    {

    }

    double SSTIOGroup::sample(int batch_idx)
    {
        // if (!m_is_signal_pushed) {
        //     throw Exception("SSTIOGroup::sample(): signal has not been pushed",
        //                     GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        // }
        // if (!m_is_batch_read) {
        //     throw Exception("SSTIOGroup::sample(): signal has not been read",
        //                     GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        // }
        // if (batch_idx != 0) {
        //     throw Exception("SSTIOGroup::sample(): batch_idx out of range",
        //                     GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        // }
        // return m_time_curr;
        if (batch_idx < 0 || batch_idx >= static_cast<int>(m_signal_pushed.size())) {
            throw Exception("SSTIOGroup::sample(): batch_idx out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        if (!m_is_read) {
            throw Exception("SSTIOGroup::sample() called before the signal was read.",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        return m_signal_pushed[batch_idx]->sample();
    }

    void SSTIOGroup::adjust(int batch_idx, double setting)
    {
        // throw Exception("SSTIOGroup::adjust(): there are no controls supported by the SSTIOGroup",
        //                 GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    double SSTIOGroup::read_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        auto idx = push_signal(signal_name, domain_type, domain_idx);
        read_batch();
        return sample(idx);
        // if (!is_valid_signal(signal_name)) {
        //     throw Exception("SSTIOGroup:read_signal(): " + signal_name +
        //                     "not valid for SSTIOGroup",
        //                     GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        // }
        // if (domain_type != GEOPM_DOMAIN_CPU) {
        //     throw Exception("SSTIOGroup::read_signal(): signal_name " + signal_name +
        //                     " not defined for domain " + std::to_string(domain_type),
        //                     GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        // }
        // return geopm_time_since(&m_time_zero);
    }

    void SSTIOGroup::write_control(const std::string &control_name, int domain_type, int domain_idx, double setting)
    {
        // throw Exception("SSTIOGroup::write_control(): there are no controls supported by the SSTIOGroup",
        //                 GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    void SSTIOGroup::save_control(void)
    {

    }

    void SSTIOGroup::restore_control(void)
    {

    }

    std::string SSTIOGroup::plugin_name(void)
    {
        return "SST";
    }

    std::unique_ptr<IOGroup> SSTIOGroup::make_plugin(void)
    {
        return geopm::make_unique<SSTIOGroup>(platform_topo(), nullptr);
    }

    std::function<double(const std::vector<double> &)> SSTIOGroup::agg_function(const std::string &signal_name) const
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("SSTIOGroup::agg_function(): " + signal_name +
                            "not valid for SSTIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return Agg::select_first;
    }

    std::function<std::string(double)> SSTIOGroup::format_function(const std::string &signal_name) const
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("SSTIOGroup::format_function(): " + signal_name +
                            "not valid for SSTIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return string_format_double;
    }

    std::string SSTIOGroup::signal_description(const std::string &signal_name) const
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("SSTIOGroup::signal_description(): " + signal_name +
                            "not valid for SSTIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        std::string result = "Invalid signal description: no description found.";
        result = "    description: Time since the start of application profiling.\n";
        result += "    units: " + IOGroup::units_to_string(M_UNITS_SECONDS) + '\n';
        result += "    aggregation: " + Agg::function_to_name(Agg::select_first) + '\n';
        result += "    domain: " + platform_topo().domain_type_to_name(GEOPM_DOMAIN_CPU) + '\n';
        result += "    iogroup: SSTIOGroup";

        return result;
    }

    std::string SSTIOGroup::control_description(const std::string &control_name) const
    {
        throw Exception("SSTIOGroup::control_description(): there are no controls supported by the SSTIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
}

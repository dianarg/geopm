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

#include "StubIOGroup.hpp"

#include <cmath>

#include <fstream>
#include <string>

#include "geopm/IOGroup.hpp"
#include "geopm/PlatformTopo.hpp"
#include "geopm/Exception.hpp"
#include "geopm/Agg.hpp"
#include "geopm/Helper.hpp"

using geopm::Exception;
using geopm::PlatformTopo;

// Registers this IOGroup with the IOGroup factory, making it visible
// to PlatformIO when the plugin is first loaded.
static void __attribute__((constructor)) example_iogroup_load(void)
{
    geopm::iogroup_factory().register_plugin(StubIOGroup::plugin_name(),
                                             StubIOGroup::make_plugin);
}

// Set up mapping between signal and control names and corresponding indices
StubIOGroup::StubIOGroup()
    : m_platform_topo(geopm::platform_topo())
    , m_is_batch_read(false)
    , m_signal_names({"ENERGY_PACKAGE", "ENERGY_DRAM",
            "CYCLES_THREAD", "CYCLES_REFERENCE",
            "FREQUENCY", "TEMPERATURE_CORE"})
{

}

// Extract the set of all signal names from the index map
std::set<std::string> StubIOGroup::signal_names(void) const
{
    return m_signal_names;
}

// Extract the set of all control names from the index map
std::set<std::string> StubIOGroup::control_names(void) const
{
    std::set<std::string> result;
    return result;
}

// Check signal name using index map
bool StubIOGroup::is_valid_signal(const std::string &signal_name) const
{
    return m_signal_names.find(signal_name) != m_signal_names.end();
}

// Check control name using index map
bool StubIOGroup::is_valid_control(const std::string &control_name) const
{
    return false;
}

int StubIOGroup::signal_domain_type(const std::string &signal_name) const
{
    int result = GEOPM_DOMAIN_INVALID;
    if (is_valid_signal(signal_name)) {
        result = GEOPM_DOMAIN_BOARD;
    }
    return result;
}

// Return board domain for all valid controls
int StubIOGroup::control_domain_type(const std::string &control_name) const
{
    int result = GEOPM_DOMAIN_INVALID;
    if (is_valid_control(control_name)) {
        result = GEOPM_DOMAIN_BOARD;
    }
    return result;
}

// Mark the given signal to be read by read_batch()
int StubIOGroup::push_signal(const std::string &signal_name, int domain_type, int domain_idx)
{
    if (!is_valid_signal(signal_name)) {
        throw Exception("StubIOGroup::push_signal(): signal_name " + signal_name +
                        " not valid for StubIOGroup.",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (domain_type != GEOPM_DOMAIN_BOARD) {
        throw Exception("StubIOGroup::push_signal(): domain_type must be M_DOMAIN_BOARD.",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(GEOPM_DOMAIN_BOARD)) {
        throw Exception("StubIOGroup::push_signal(): domain_idx out of range.",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (m_is_batch_read) {
        throw Exception("StubIOGroup::push_signal(): cannot push signal after call to read_batch().",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return 0;
}

// Mark the given control to be written by write_batch()
int StubIOGroup::push_control(const std::string &control_name, int domain_type, int domain_idx)
{
    return -1;
}
// Parse /proc/stat and update saved values for signals
void StubIOGroup::read_batch(void)
{
    m_is_batch_read = true;
}


void StubIOGroup::write_batch(void)
{
}

// Return the latest value read by read_batch()
double StubIOGroup::sample(int batch_idx)
{
    if (batch_idx != 0) {
        throw Exception("StubIOGroup::sample(): batch_idx out of range.",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (!m_is_batch_read) {
        throw Exception("StubIOGroup::sample(): signal has not been read.",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return NAN;
}

// Save a setting to be written by a future write_batch()
void StubIOGroup::adjust(int batch_idx, double setting)
{

}

// Read the value of a signal immediately, bypassing read_batch()
double StubIOGroup::read_signal(const std::string &signal_name, int domain_type, int domain_idx)
{
    if (!is_valid_signal(signal_name)) {
        throw Exception("StubIOGroup:read_signal(): " + signal_name +
                        "not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (domain_type != GEOPM_DOMAIN_BOARD) {
        throw Exception("StubIOGroup::push_signal(): domain_type must be M_DOMAIN_BOARD.",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (domain_idx < 0 || domain_idx >= m_platform_topo.num_domain(GEOPM_DOMAIN_BOARD)) {
        throw Exception("StubIOGroup::push_signal(): domain_idx out of range.",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return NAN;
}

// Write to the control immediately, bypassing write_batch()
void StubIOGroup::write_control(const std::string &control_name, int domain_type, int domain_idx, double setting)
{
}

// Implemented to allow an IOGroup platform settings before starting
// to adjust them
void StubIOGroup::save_control(void)
{

}

// Implemented to allow an IOGroup to restore previously saved
// platform settings
void StubIOGroup::restore_control(void)
{

}

// Hint to Agent about how to aggregate signals from this IOGroup
std::function<double(const std::vector<double> &)> StubIOGroup::agg_function(const std::string &signal_name) const
{
    if (!is_valid_signal(signal_name)) {
        throw Exception("StubIOGroup:agg_function(): " + signal_name +
                        "not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    // All signals will be aggregated as an average
    return geopm::Agg::average;
}

// Specifies how to print signals from this IOGroup
std::function<std::string(double)> StubIOGroup::format_function(const std::string &signal_name) const
{
    if (!is_valid_signal(signal_name)) {
        throw Exception("StubIOGroup::format_function(): " + signal_name +
                        "not valid for TimeIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return geopm::string_format_double;
}

// A user-friendly description of each signal
std::string StubIOGroup::signal_description(const std::string &signal_name) const
{
    if (!is_valid_signal(signal_name)) {
        throw Exception("StubIOGroup::signal_description(): signal_name " + signal_name +
                        " not valid for StubIOGroup.",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return "Stub signal; always returns NAN";
}

// A user-friendly description of each control
std::string StubIOGroup::control_description(const std::string &control_name) const
{
    return "";
}

// Name used for registration with the IOGroup factory
std::string StubIOGroup::plugin_name(void)
{
    return "stub";
}

// Function used by the factory to create objects of this type
std::unique_ptr<geopm::IOGroup> StubIOGroup::make_plugin(void)
{
    return std::unique_ptr<geopm::IOGroup>(new StubIOGroup);
}

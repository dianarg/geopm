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

#ifndef PLATFORMIO_HPP_INCLUDE
#define PLATFORMIO_HPP_INCLUDE

#include <stdint.h>
#include <string>
#include <vector>
#include <map>

namespace geopm
{
    /// @brief Class which is a collection of all valid control and
    /// signal objects for a platform
    class IPlatformIO
    {
        public:
            enum m_group_e {
                M_GROUP_BOARD,
                M_GROUP_PACKAGE,
                M_GROUP_NUMA,
                M_GROUP_L3CACHE,
                M_GROUP_L2CACHE,
                M_GROUP_L1CACHE,
                M_GROUP_EXT_BASE = 2048,
            };

            enum m_component_e {
                M_COMPONENT_CPU,
                M_COMPONENT_MEMORY,
                M_COMPONENT_PMEMORY,
                M_COMPONENT_NETWORK,
                M_COMPONENT_OTHER,
            };

            IPlatformIO() {}
            virtual ~IPlatformIO() {}
            /// @brief Returns a domain given the group and component
            virtual uint64_t group_component_to_domain(uint32_t group, uint32_t component) const = 0;
            /// @brief Returns a group/component pair given a domain
            virtual std::pair<uint32_t, uint32_t> domain_to_group_component(uint64_t dom) const = 0;
            /// @brief Add components to one of the etc groups.
            virtual void group_ext_define(int ext_idx, const std::vector<std::set<int> > &cpu_idx) = 0;
            /// @brief Remove all components from one of the etc groups.
            virtual void group_ext_clear(int ext_idx) = 0;
            /// @brief Number of domains on the platform of a
            ///        particular m_domain_e type.
            virtual int num_domain(uint64_t domain_type) const = 0;
            /// @brief Get the set of Linux logical CPUs associated with the indexed domain
            virtual void domain_cpus(uint64_t domain_type, int domain_idx, std::set<int> &cpu_idx) const = 0;
            /// @brief Get the domain index for a particular domain
            ///        type that contains the given Linux logical CPU
            ///        index.
            int domain_idx(uint64_t domain_type, int cpu_idx) const = 0;
            /// @brief Push a signal onto the end of the vector that
            ///        can be sampled.
            /// @param [in] signal_name Name of the signal requested.
            /// @param [in] domain_type One of the values from the
            ///        geopm_domain_type_e enum described in
            ///        PlatformTopology.hpp.
            /// @param [in] domain_idx The index of the domain within
            ///        the set of domains of the same type on the
            ///        platform.
            /// @return Index of signal when sample() method is called
            ///         or -1 if the signal is not valid on the
            ///         platform.
            virtual int push_signal(const std::string &signal_name,
                                    uint64_t domain_type,
                                    int domain_idx) = 0;
            /// @brief Push a control onto the end of the vector that
            ///        can be adjusted.
            /// @param [in] control_name Name of the control requested.
            /// @param [in] domain_type One of the values from the
            ///        geopm_domain_type_e enum described in
            ///        PlatformTopology.hpp.
            /// @param [in] domain_idx The index of the domain within
            ///        the set of domains of the same type on the
            ///        platform.
            /// @return Pointer to an IControl object if the requested
            ///         control is valid, otherwise returns NULL.
            virtual int push_control(const std::string &control_name,
                                     uint64_t domain_type,
                                     int domain_idx) = 0;
            /// @brief Remove all signals and controls.  Must be
            ///        called before pushing signals or controls once
            ///        they have been sampled or adjusted.
            virtual void clear(void) = 0;
            /// @brief Sample a single signal that has been pushed on
            ///        to the signal stack.
            /// @param [in] signal_idx index returned by a previous call
            ///        to push_signal() method.
            virtual double sample(int signal_idx) = 0;
            /// @brief Format the value of the signal into a printable
            ///        form including units.
            /// @param signal_idx index returned by a previous call
            ///        to push_signal() method.
            /// @return Printable version of the signal suited for
            ///         output in a log file.
            virtual std::string log(int signal_idx, double sample) = 0;
            /// @brief Adjust a single control that has been pushed on
            ///        to the control stack.
            /// @param [in] control_idx index returned by a previous call
            ///        to push_control() method.
            virtual void adjust(int control_idx,
                                double setting) = 0;
            /// @brief Measure the signals specified by the previous
            ///        call to the config() method.
            /// @param [out] signal Vector of signal values measured
            ///        from the platform in SI units.  The order of
            ///        these signals is determined by the previous
            ///        call to the config() method.
            virtual void sample(std::vector<double> &signal) = 0;
            /// @brief Set values of controls specified by the previous
            ///        calls to push_control().
            /// @brief [in] control_idx Index of control to be
            ///        adjusted.
            /// @param [in] setting Vector of control parameter values
            ///        in SI units.  The order of these controls is
            ///        determined by the previous call to the config()
            ///        method.
            virtual void adjust(const std::vector<double> &setting) = 0;
            /// @brief Fill string with the msr-safe whitelist file contents
            ///        reflecting all known MSRs for the current platform.
            /// @return String formatted to be written to
            ///        an msr-safe whitelist file.
            virtual std::string msr_whitelist(void) const = 0;
            /// @brief Fill string with the msr-safe whitelist file
            ///        contents reflecting all known MSRs for the
            ///        specified platform.
            /// @param cpuid [in] The CPUID of the platform.
            /// @return String formatted to be written to an msr-safe
            ///         whitelist file.
            virtual std::string msr_whitelist(int cpuid) const = 0;
    };

    IPlatformIO &platform_io(void);

    /// @brief Abstract base class describing a signal provided by a
    /// platform that can be sampled.
    class ISignal
    {
        public:
            ISignal() {}
            virtual ~ISignal() {}
            /// @brief Get the signal parameter name.
            /// @param [out] signal_name The name of the feature
            ///        being measured.
            virtual std::string name(void) const = 0;
            /// @brief Get the type of the domain under measurement.
            /// @return One of the values from the IPlatformIO::m_domain_e
            ///         enum described in PlatformTopology.hpp.
            virtual uint64_t domain_type(void) const = 0;
            /// @brief Get the index of the domain under measurement.
            /// @return The index of the domain within the set of
            ///        domains of the same type on the platform.
            virtual int domain_idx(void) const = 0;
            /// @brief Get the value of the signal.
            /// @return The value of the parameter measured in SI
            ///         units.
            virtual double sample(void) const = 0;
            /// @brief Format the value of the signal into a printable
            ///        form including units.
            /// @param sample output of a previous call to the
            ///        sample() method.
            /// @return Printable version of the signal suited for
            ///         output in a log file.
            virtual std::string log(double sample) const = 0;
    };

    /// @brief Abstract base class describing a control provided by a
    /// platform that can be adjusted.
    class IControl
    {
        public:
            IControl() {}
            virtual ~IControl() {}
            /// @brief Get the control parameter name.
            /// @param [out] control_name The name of the feature
            ///        under control.
            virtual std::string name(void) const = 0;
            /// @brief Get the type of the domain under control.
            /// @return One of the values from the geopm_domain_type_e
            ///         enum described in PlatformTopology.hpp.
            virtual uint64_t domain_type(void) const = 0;
            /// @brief Get the index of the domain under control.
            /// @return The index of the domain within the set of
            ///        domains of the same type on the platform.
            virtual int domain_idx(void) const = 0;
            /// @brief Set the value for the control.
            /// @param [in] setting value in SI units of the parameter
            ///        controlled by the object.
            virtual void adjust(double setting) = 0;
    };

}

#endif

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

#include <vector>
#include <map>
#include <string>

namespace geopm {

    enum aggregation_op_type_e {
        AGGREGATION_OP_SUM,
        AGGREGATION_OP_AVG,
        AGGREGATION_OP_MIN,
        AGGREGATION_OP_MAX,
    };

    class TelemetryConfig {
        public:
            /// @brief TelemetryConfig constructor.
            ///
            /// @param [in] fan_out Fan out of the communnications tree.
            TelemetryConfig(const std::vector<int> &fan_out);
            /// @brief TelemetryConfig copy constructor.
            ///
            /// @param [in] other TelemetryConfig object to copy.
            TelemetryConfig(const TelemetryConfig &other);
            /// @brief destructor.
            virtual ~TelemetryConfig();
            /// @brief Set what signals are provided.
            ///
            /// It is expected that the Platform will set all signals it supports using
            /// this interface. It should be called before any other entity besides the platform
            /// attempts to use the object.
            ///
            /// @param [in] signal_domain The signal domain the signals belong to.
            ///
            /// @param [in] provided The names of the supported signals for the specified domain.
            void set_provided(int signal_domain, const std::vector<std::string> &provided);
            /// @brief Get the list of supported signals.
            ///
            /// The deciders should call this function to get a list of the signals
            /// supported by the current platform.
            ///
            /// @param [in] signal_domain The signal domain the signals belong to.
            ///
            /// @param [in] provided The names of the supported signals for the specified domain.
            void get_provided(int signal_domain, std::vector<std::string> &provided) const;
            /// @brief Query if a specific signal is supported.
            ///
            /// A decider can query if a signal of interest is supported by the current platform.
            ///
            /// @param [in] signal_domain The signal domain the signal belong to.
            ///
            /// @param [in] The signal of interest to query for.
            ///
            /// @return Returns true if the signal is suppotred, else returns false.
            bool is_provided(int signal_domain, const std::string &signal) const;
            /// @brief Set what signals are required.
            ///
            /// It is expected that the leaf level deciders will set all signals
            /// that they requre using this interface.
            ///
            /// @param [in] signal_domain The signal domain the signals belong to.
            ///
            /// @param [in] required The names of the requested signals for the specified domain.
            void set_required(int signal_domain, const std::vector<std::string> &required);
            /// @brief Set a required signal.
            ///
            /// A decider can request a single signal using this interface.
            ///
            /// @param [in] signal_domain The signal domain the signals belong to.
            ///
            /// @param [in] required The name of the requested signal for the specified domain.
            void set_required(int signal_domain, const std::string &required);
            /// @brief Get the list of requested signals.
            ///
            /// The platform and other agents should use this to query the signals that
            /// will be used during the run.for the specified signal domain.
            ///
            /// @param [in] signal_domain The signal domain the signals belong to.
            ///
            /// @param [in] required The names of the requested signals for the specified domain.
            void get_required(int signal_domain, std::vector<std::string> &required) const;
            /// @brief Get the list of requested signals.
            ///
            /// The platform and other agents should use this to query all signals that
            /// will be used during the run.
            ///
            /// @param [in] signal_domain The signal domain the signals belong to.
            ///
            /// @param [in] required The names of the requested signals for the specified domain
            ///        per signal domain.
            void get_required(std::map<int, std::vector<std::string> > &required) const;
            /// @brief Query if a specific signal has been requested.
            ///
            /// A platform oother agent can query if a signal of interest has been requested by a decider.
            ///
            /// @param [in] signal_domain The signal domain the signal belong to.
            ///
            /// @param [in] The signal of interest to query for.
            ///
            /// @return Returns true if the signal is requested, else returns false.
            bool is_required(int signal_domain, const std::string &signal) const;
            /// @brief Create map between signal domain and linux cpu.
            ///
            /// It is expected that the Platform will create maps for all signal domains it supports using
            /// this interface. It should be called before any other entity besides the platform
            /// attempts to use the object.
            ///
            /// @param [in] signal_domain The signal domain the signals belong to.
            ///
            /// @param [in] provided A map between an entry on the signal domain and the list of
            ///        Linux cpu ids that belong to that entry.
            void set_domain_cpu_map(int domain, const std::vector<std::vector<int> > &domain_map);
            /// @brief Retieve a map between a signal domain and linux cpus.
            ///
            /// It is expected that the leaf level decider will retrieve maps for all signal domains it
            /// is interested in using this interface.
            ///
            /// @param [in] signal_domain The signal domain the signals belong to.
            ///
            /// @param [out] domain_map A map between an entry on the signal domain and the list of
            ///        Linux cpu ids that belong to that entry.
            void get_domain_cpu_map(int domain, std::vector<std::vector<int> > &domain_map) const;
            /// @brief retrieve the number of sub-domain signals per signal domain.
            ///
            /// Retrieve an integer per signal domain that describes the number of sub-domain entities that are
            /// contained within the domain. As an example, if a signal domain was at the CPU level, it would
            /// give the number of CPUs on the system.
            ///
            /// @param num_signal the number of sub-domain signals for each signal domain.
            void num_signal_per_domain(std::vector<int> &num_signal) const;
            /// @brief retrieve the number of sub-domain signals for a specific signal domain.
            ///
            /// Retrieve an integer for a specified signal domain that describes the number of sub-domain entities
            /// that are contained within the domain. As an example, if the signal domain was at the CPU level, it
            /// would give the number of CPUs on the system.
            ///
            /// @param num_signal the number of sub-domain signals for the specified signal domain.
            int num_signal_per_domain(int domain) const;
            /// @brief Retrieve the total number of requested signals.
            ///
            /// @return Returns the total number of requested signals across all signal domains.
            int num_required_signal(void) const;
            /// @brief Set the minimum and maximum control values for a control domain.
            ///
            /// Set the control bounds for a specific control domain. For example, if the specified
            /// control domain is frequency, it would set the min and max supported p-states. It is
            /// expected that the Platform will call this for every supported control type.
            ///
            /// @param [in] control_domain The control domain of interest.
            ///
            /// @param [in] lower The minimum control vaule bound.
            ///
            /// @param [in] lower The maximum control vaule bound.
            void set_bounds(int control_domain, double lower, double upper);
            /// @brief Get the minimum and maximum control values for a control domain.
            ///
            /// Get the control bounds for a specific control domain. For example, if the specified
            /// control domain is frequency, it would return the min and max supported p-states. It is
            /// expected that the Deciders will call this for every control type it plans to modify.
            ///
            /// @param [in] control_domain The control domain of interest.
            ///
            /// @param [out] lower The minimum control vaule bound.
            ///
            /// @param [out] lower The maximum control vaule bound.
            void get_bounds(int level, int ctl_domain, double &lower, double &upper) const;
            /// @brief Set the signal and control domains that are supported.
            ///
            /// It is expected that the Platform will specify all signal and control domains it supports
            /// using this interface. These are specified as geopm_domain_e enums. It should be called before
            /// any other entity besides the platform attempts to use the object.
            ///
            /// @param [in] domain The signal and control domains supported by the system.
            ///        Linux cpu ids that belong to that entry.
            void supported_domain(const std::vector<int> domain);
            /// @brief Query if a specific signal or control domain is supported.
            ///
            /// It is expected that the Deciders will query if all required signal and control domains
            /// are supported using this interface. These are specified as geopm_domain_e enums.
            ///
            /// @param [in] domain The signal or control domain of interest.
            ///
            /// @return Returns true if the specified domain is supperted, else returns false.
            bool is_supported_domain(int domain) const;
            /// @brief Set which signals are required to be aggregated.
            ///
            /// It is expected that the tree level deciders will set all signals
            /// that they require using this interface. They will then be aggregated by the level below
            /// using the the spacial and temporal operations specified. The supported operations are
            /// defined in enum aggregation_op_type_e.
            ///
            /// @param [in] agg The names and spacial/temporal operations of the requested signals.
            void set_aggregate(const std::vector<std::pair<std::string, std::pair<int, int> > > &agg);
            /// @brief Set a signals as required to be aggregated.
            ///
            /// It is expected that the tree level deciders can set a specific signal to be aggregated
            /// using this interface. The specified signal will then be aggregated by the level below
            /// using the the spacial and temporal operations specified. The supported operations are
            /// defined in enum aggregation_op_type_e.
            ///
            /// @param [in] agg The names of the requested signal.
            ///
            /// @param [in] spacial_op_type the aggreagation operation across the spacial domain.
            ///
            /// @param [in] temporal_op_type the aggreagation operation across the temporal domain.
            void set_aggregate(std::string signal, int spacial_op_type, int temporal_op_type);
            /// @brief Get which signals are required to be aggregated.
            ///
            /// It is expected that the Region objects will get all signals that are required to be
            /// aggregated using this interface. They will then aggregate the requested signals and provide them
            /// as a sample message to be sent to the next (upper) level of the tree.. The supported operations are
            /// defined in enum aggregation_op_type_e.
            ///
            /// @param [out] agg The names and spacial/temporal operations of the requested signals.
            void get_aggregate(std::vector<std::pair<std::string, std::pair<int, int> > > &agg) const;
            /// @brief The number of signals to be aggregated.
            ///
            /// @return Returns the total number of signals to be aggregated.
            int num_aggregated_signal(void) const;
        private:
            int num_children(int level);
            std::map<int, std::vector<std::string> > m_provided_signal;
            std::map<int, std::vector<std::string> > m_required_signal;
            std::vector<std::pair<std::string, std::pair<int, int> > > m_aggregate_signal;
            std::map<int, std::pair<double, double> > m_control_bound;
            std::map<int, std::vector<std::vector<int> > > m_domain_map;
            std::vector<int> m_supported_domain;
            std::vector<int> m_fan_out;
    };
}
#endif

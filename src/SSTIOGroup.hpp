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

#ifndef SSTIOGROUP_HPP_INCLUDE
#define SSTIOGROUP_HPP_INCLUDE

#include <set>
#include <functional>

#include "IOGroup.hpp"


namespace geopm
{
    class PlatformTopo;
    class SSTIO;
    class Signal;
    class Control;

    /// @brief IOGroup that provides a signal for the time since GEOPM startup.
    class SSTIOGroup : public IOGroup
    {
        public:
            SSTIOGroup(const PlatformTopo &topo, std::shared_ptr<SSTIO> sstio);
            virtual ~SSTIOGroup() = default;
            std::set<std::string> signal_names(void) const override;
            std::set<std::string> control_names(void) const override;
            bool is_valid_signal(const std::string &signal_name) const override;
            bool is_valid_control(const std::string &control_name) const override;
            int signal_domain_type(const std::string &signal_name) const override;
            int control_domain_type(const std::string &control_name) const override;
            int push_signal(const std::string &signal_name, int domain_type, int domain_idx)  override;
            int push_control(const std::string &control_name, int domain_type, int domain_idx) override;
            void read_batch(void) override;
            void write_batch(void) override;
            double sample(int batch_idx) override;
            void adjust(int batch_idx, double setting) override;
            double read_signal(const std::string &signal_name, int domain_type, int domain_idx) override;
            void write_control(const std::string &control_name, int domain_type, int domain_idx, double setting) override;
            void save_control(void) override;
            void restore_control(void) override;
            std::function<double(const std::vector<double> &)> agg_function(const std::string &signal_name) const override;
            std::function<std::string(double)> format_function(const std::string &signal_name) const override;
            std::string signal_description(const std::string &signal_name) const override;
            std::string control_description(const std::string &control_name) const override;
            static std::string plugin_name(void);
            static std::unique_ptr<IOGroup> make_plugin(void);
        private:
            bool m_is_signal_pushed;
            bool m_is_batch_read;
            const std::set<std::string> m_valid_signal_name;
            /////// old above

            const PlatformTopo &m_topo;
            std::shared_ptr<SSTIO> m_sstio;
            bool m_is_read;

            // All available signals: map from name to signal_info.
            // The signals vector is over the indices for the domain.
            // The signals pointers should be copied when signal is
            // pushed and used directly for read_signal.
            struct signal_info
            {
                std::vector<std::shared_ptr<Signal> > signals;
                int domain;
                int units;
                std::function<double(const std::vector<double> &)> agg_function;
                std::string description;
            };
            std::map<std::string, signal_info> m_signal_available;

            // Mapping of signal index to pushed signals.
            std::vector<std::shared_ptr<Signal> > m_signal_pushed;

            // Mapping of control index to pushed controls
            std::vector<std::shared_ptr<Control> > m_control_pushed;
    };
}

#endif

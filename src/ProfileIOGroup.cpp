/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
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

#include "ProfileIOGroup.hpp"

#include "PlatformTopo.hpp"
#include "ProfileEventBuffer.hpp"
#include "Helper.hpp"
#include "Exception.hpp"
#include "Agg.hpp"
#include "geopm_hash.h"
#include "geopm_time.h"
#include "geopm_internal.h"
#include "config.h"

#define GEOPM_PROFILE_IO_GROUP_PLUGIN_NAME "PROFILE"

namespace geopm
{
    class ProfileEpoch
    {
        public:
            ProfileEpoch() = default;
            virtual ~ProfileEpoch() = default;
    };

    class ProfileRegion
    {
        public:
            ProfileRegion() = default;
            virtual ~ProfileRegion() = default;
            uint64_t hash(void);
            double progress(void);
            int count(void);
            double runtime(void);
        private:
            uint64_t m_hash;
    };

    uint64_t ProfileRegion::hash(void)
    {
        return m_hash;
    }

    double ProfileRegion::progress(void)
    {
        return 0.0;
    }

    int ProfileRegion::count(void)
    {
        return 0;
    }

    double ProfileRegion::runtime(void)
    {
        return 0.0;
    }

    class ProfileRank
    {
        public:
            ProfileRank(const ProfileEventBuffer &profile_event_buffer,
                        int local_rank);
            virtual ~ProfileRank() = default;
            void update(void);
            int epoch_count(void);
            double epoch_runtime(void);
            double epoch_runtime_network(void);
            double epoch_runtime_ignore(void);
            uint64_t region_hash(void);
            double region_progress(void);
            int region_count(void);
            double region_runtime(void);
        private:
            const ProfileEventBuffer &m_profile_event_buffer;
            ProfileEventQuery m_query;
            uint64_t m_current_hash;
            std::map<uint64_t, ProfileRegion> m_regions;
            ProfileEpoch m_epoch;
    };

    ProfileRank::ProfileRank(const ProfileEventBuffer &profile_event_buffer,
                             int local_rank)
        : m_profile_event_buffer(profile_event_buffer)
        , m_query(local_rank)
        , m_current_hash(m_profile_event_buffer.hash_begin())
    {

    }

    void ProfileRank::update(void)
    {
        m_query.update_serial(m_profile_event_buffer.serial_end());

    }

    int ProfileRank::epoch_count(void)
    {
        return 0;
    }

    double ProfileRank::epoch_runtime(void)
    {
        return 0.0;
    }

    double ProfileRank::epoch_runtime_network(void)
    {
        return 0.0;
    }

    double ProfileRank::epoch_runtime_ignore(void)
    {
        return 0.0;
    }

    uint64_t ProfileRank::region_hash(void)
    {
        return m_current_hash;
    }

    double ProfileRank::region_progress(void)
    {
        return 0.0;
    }

    int ProfileRank::region_count(void)
    {
        return 0;
    }

    double ProfileRank::region_runtime(void)
    {
        return 0.0;
    }

    ProfileIOGroup::ProfileIOGroup()
        : ProfileIOGroup(platform_topo(),
                         profile_event_buffer())
    {

    }

    ProfileIOGroup::ProfileIOGroup(const PlatformTopo &topo,
                                   ProfileEventBuffer &profile_event_buffer)
        : m_platform_topo(topo)
        , m_profile_event_buffer(profile_event_buffer)
        , m_signal_idx_map{{plugin_name() + "::REGION_HASH", M_SIGNAL_REGION_HASH},
                           {plugin_name() + "::REGION_HINT", M_SIGNAL_REGION_HINT},
                           {plugin_name() + "::REGION_PROGRESS", M_SIGNAL_REGION_PROGRESS},
                           {plugin_name() + "::REGION_COUNT", M_SIGNAL_REGION_COUNT},
                           {plugin_name() + "::REGION_THREAD_PROGRESS", M_SIGNAL_THREAD_PROGRESS},
                           {"REGION_HASH", M_SIGNAL_REGION_HASH},
                           {"REGION_HINT", M_SIGNAL_REGION_HINT},
                           {"REGION_PROGRESS", M_SIGNAL_REGION_PROGRESS},
                           {"REGION_COUNT", M_SIGNAL_REGION_COUNT},
                           {"REGION_THREAD_PROGRESS", M_SIGNAL_THREAD_PROGRESS},
                           {plugin_name() + "::EPOCH_RUNTIME", M_SIGNAL_EPOCH_RUNTIME},
                           {"EPOCH_RUNTIME", M_SIGNAL_EPOCH_RUNTIME},
                           {plugin_name() + "::EPOCH_COUNT", M_SIGNAL_EPOCH_COUNT},
                           {"EPOCH_COUNT", M_SIGNAL_EPOCH_COUNT},
                           {plugin_name() + "::REGION_RUNTIME", M_SIGNAL_REGION_RUNTIME},
                           {"REGION_RUNTIME", M_SIGNAL_REGION_RUNTIME},
                           {plugin_name() + "::EPOCH_RUNTIME_NETWORK", M_SIGNAL_EPOCH_RUNTIME_NETWORK},
                           {"EPOCH_RUNTIME_NETWORK", M_SIGNAL_EPOCH_RUNTIME_NETWORK},
                           {plugin_name() + "::EPOCH_RUNTIME_IGNORE", M_SIGNAL_EPOCH_RUNTIME_IGNORE},
                           {"EPOCH_RUNTIME_IGNORE", M_SIGNAL_EPOCH_RUNTIME_IGNORE}}
        , m_do_read_batch(M_SIGNAL_MAX, false)
    {

    }

    ProfileIOGroup::~ProfileIOGroup()
    {

    }

    std::set<std::string> ProfileIOGroup::signal_names(void) const
    {
        std::set<std::string> result;
        for (const auto &sv : m_signal_idx_map) {
            result.insert(sv.first);
        }
        return result;
    }

    std::set<std::string> ProfileIOGroup::control_names(void) const
    {
        return {};
    }

    bool ProfileIOGroup::is_valid_signal(const std::string &signal_name) const
    {
        return m_signal_idx_map.find(signal_name) != m_signal_idx_map.end();
    }

    bool ProfileIOGroup::is_valid_control(const std::string &control_name) const
    {
        return false;
    }

    int ProfileIOGroup::signal_domain_type(const std::string &signal_name) const
    {
        int result = GEOPM_DOMAIN_INVALID;
        if (is_valid_signal(signal_name)) {
            result = GEOPM_DOMAIN_CPU;
        }
        return result;
    }

    int ProfileIOGroup::control_domain_type(const std::string &control_name) const
    {
        return GEOPM_DOMAIN_INVALID;
    }

    int ProfileIOGroup::push_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        int result = -1;
        int signal_type = check_signal(signal_name, domain_type, domain_idx);

        int signal_idx = 0;
        for (const auto &it : m_active_signal) {
            if (it.signal_type == signal_type &&
                it.domain_type == domain_type &&
                it.domain_idx == domain_idx) {
                result = signal_idx;
            }
            ++signal_idx;
        }
        if (result == -1) {
            result = m_active_signal.size();
            m_active_signal.push_back({signal_type, domain_type, domain_idx});
            m_do_read_batch[signal_type] = true;
            // Runtime and count signals require region hash signal to be sampled
            if (signal_type == M_SIGNAL_REGION_RUNTIME ||
                signal_type == M_SIGNAL_REGION_COUNT) {
                m_do_read_batch[M_SIGNAL_REGION_HASH] = true;
            }
        }
        return result;
    }

    int ProfileIOGroup::push_control(const std::string &control_name, int domain_type, int domain_idx)
    {
        throw Exception("ProfileIOGroup::push_control() there are no controls supported by the ProfileIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    void ProfileIOGroup::init_batch(void)
    {
        if (m_profile_rank.size() == 0) {
            m_cpu_rank = m_profile_event_buffer.cpu_rank();
            std::set<int> global_rank;
            for (auto &rank : m_cpu_rank) {
                global_rank.insert(rank);
            }
            int num_node_rank = global_rank.size();
            m_global_rank.resize(num_node_rank);
            int idx = 0;
            for (auto &rank : global_rank) {
                m_global_rank[idx] = rank;
                std::vector<int> signal_type;
                for (auto as : m_active_signal) {
                    if (m_cpu_rank[as.domain_idx] == rank) {
                        signal_type.push_back(as.signal_type);
                    }
                }
                m_profile_rank.emplace_back(m_profile_event_buffer, idx);
                ++idx;
            }
        }
    }
    void ProfileIOGroup::read_batch(void)
    {
        init_batch();
        for (auto &pr : m_profile_rank) {
            pr.update();
        }
        throw Exception("ProfileIOGroup: Implementation using ProfileEventBuffer is incomplete",
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }

    void ProfileIOGroup::write_batch(void)
    {

    }

    double ProfileIOGroup::sample(int signal_idx)
    {
        double result = NAN;
        if (signal_idx < 0 || signal_idx >= (int)m_active_signal.size()) {
            throw Exception("ProfileIOGroup::sample(): signal_idx out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        throw Exception("ProfileIOGroup: Implementation using ProfileEventBuffer is incomplete",
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
        return result;
    }

    void ProfileIOGroup::adjust(int control_idx, double setting)
    {
        throw Exception("ProfileIOGroup::adjust() there are no controls supported by the ProfileIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    double ProfileIOGroup::read_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        double result = NAN;
        throw Exception("ProfileIOGroup::read_signal is not supported, use batch interface instead",
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
        return result;
    }

    void ProfileIOGroup::write_control(const std::string &control_name, int domain_type, int domain_idx, double setting)
    {
        throw Exception("ProfileIOGroup::write_control() there are no controls supported by the ProfileIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    void ProfileIOGroup::save_control(void)
    {

    }

    void ProfileIOGroup::restore_control(void)
    {

    }

    std::function<double(const std::vector<double> &)> ProfileIOGroup::agg_function(const std::string &signal_name) const
    {
        static const std::map<std::string, std::function<double(const std::vector<double> &)> > fn_map {
            {"REGION_RUNTIME", Agg::max},
            {"PROFILE::REGION_RUNTIME", Agg::max},
            {"REGION_PROGRESS", Agg::min},
            {"PROFILE::REGION_PROGRESS", Agg::min},
            {"REGION_THREAD_PROGRESS", Agg::min},
            {"PROFILE::REGION_THREAD_PROGRESS", Agg::min},
            {"REGION_HASH", Agg::region_hash},
            {"PROFILE::REGION_HASH", Agg::region_hash},
            {"REGION_HINT", Agg::region_hint},
            {"PROFILE::REGION_HINT", Agg::region_hint},
            {"REGION_COUNT", Agg::min},
            {"PROFILE::REGION_COUNT", Agg::min},
            {"EPOCH_RUNTIME", Agg::max},
            {"PROFILE::EPOCH_RUNTIME", Agg::max},
            {"EPOCH_ENERGY", Agg::sum},
            {"PROFILE::EPOCH_ENERGY", Agg::sum},
            {"EPOCH_COUNT", Agg::min},
            {"PROFILE::EPOCH_COUNT", Agg::min},
            {"EPOCH_RUNTIME_NETWORK", Agg::max},
            {"PROFILE::EPOCH_RUNTIME_NETWORK", Agg::max},
            {"EPOCH_RUNTIME_IGNORE", Agg::max},
            {"PROFILE::EPOCH_RUNTIME_IGNORE", Agg::max}
        };
        auto it = fn_map.find(signal_name);
        if (it == fn_map.end()) {
            throw Exception("ProfileIOGroup::agg_function(): unknown how to aggregate \"" + signal_name + "\"",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return it->second;
    }

    std::function<std::string(double)> ProfileIOGroup::format_function(const std::string &signal_name) const
    {
       static const std::map<std::string, std::function<std::string(double)> > fmt_map {
            {"REGION_RUNTIME", string_format_double},
            {"REGION_COUNT", string_format_integer},
            {"PROFILE::REGION_RUNTIME", string_format_double},
            {"REGION_PROGRESS", string_format_float},
            {"PROFILE::REGION_COUNT", string_format_integer},
            {"PROFILE::REGION_PROGRESS", string_format_float},
            {"REGION_THREAD_PROGRESS", string_format_float},
            {"PROFILE::REGION_THREAD_PROGRESS", string_format_float},
            {"REGION_HASH", string_format_hex},
            {"PROFILE::REGION_HASH", string_format_hex},
            {"REGION_HINT", string_format_hex},
            {"PROFILE::REGION_HINT", string_format_hex},
            {"EPOCH_RUNTIME", string_format_double},
            {"PROFILE::EPOCH_RUNTIME", string_format_double},
            {"EPOCH_ENERGY", string_format_double},
            {"PROFILE::EPOCH_ENERGY", string_format_double},
            {"EPOCH_COUNT", string_format_integer},
            {"PROFILE::EPOCH_COUNT", string_format_integer},
            {"EPOCH_RUNTIME_NETWORK", string_format_double},
            {"PROFILE::EPOCH_RUNTIME_NETWORK", string_format_double},
            {"EPOCH_RUNTIME_IGNORE", string_format_double},
            {"PROFILE::EPOCH_RUNTIME_IGNORE", string_format_double}
        };
        auto it = fmt_map.find(signal_name);
        if (it == fmt_map.end()) {
            throw Exception("ProfileIOGroup::format_function(): unknown how to format \"" + signal_name + "\"",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return it->second;
    }


    std::string ProfileIOGroup::signal_description(const std::string &signal_name) const
    {
        return "";
    }

    std::string ProfileIOGroup::control_description(const std::string &control_name) const
    {
        return "";
    }

    int ProfileIOGroup::check_signal(const std::string &signal_name, int domain_type, int domain_idx) const
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("ProfileIOGroup::check_signal(): signal_name " + signal_name +
                            " not valid for ProfileIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (domain_type != GEOPM_DOMAIN_CPU) {
            /// @todo Add support for non-cpu domains.
            throw Exception("ProfileIOGroup::check_signal(): non-CPU domains are not supported",
                            GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
        }
        int cpu_idx = domain_idx;
        if (cpu_idx < 0 || cpu_idx >= m_platform_topo.num_domain(GEOPM_DOMAIN_CPU)) {
            throw Exception("ProfileIOGroup::check_signal(): domain index out of range",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        int signal_type = -1;
        auto it = m_signal_idx_map.find(signal_name);
        if (it != m_signal_idx_map.end()) {
            signal_type = it->second;
        }
#ifdef GEOPM_DEBUG
        else {
            throw Exception("ProfileIOGroup::check_signal: is_valid_signal() returned true, but signal name is unknown",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        return signal_type;
    }

    std::string ProfileIOGroup::plugin_name(void)
    {
        return GEOPM_PROFILE_IO_GROUP_PLUGIN_NAME;
    }

    std::unique_ptr<IOGroup> ProfileIOGroup::make_plugin(void)
    {
        return geopm::make_unique<ProfileIOGroup>();
    }

}

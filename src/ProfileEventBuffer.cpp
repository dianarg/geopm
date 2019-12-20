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
#include "config.h"
#include "ProfileEventBuffer.hpp"
#include "geopm_time.h"
#include "geopm_internal.h"

namespace geopm
{
    class ProfileEvent
    {
        public:
            enum m_event_e {
                M_EVENT_EPOCH,
                M_EVENT_ENTRY,
                M_EVENT_EXIT,
                M_EVENT_PROGRESS,
            };
            ProfileEvent(const struct geopm_prof_message_s &prof_msg);
            int rank(void);
            struct geopm_time_s time(void);
            int event(void);
            uint64_t hash(void);
            uint64_t hint(void);
            double progress(void);
        private:
            struct geopm_prof_message_s m_prof_msg;
    };


    ProfileEventQuery::ProfileEventQuery(int rank)
        : m_rank(rank)
        , m_serial_begin(0)
        , m_serial_end(0)
    {

    }

    void ProfileEventQuery::update_serial(size_t serial_end)
    {
        m_serial_begin = m_serial_end;
        m_serial_end = serial_end;
    }

    bool ProfileEventQuery::operator == (const ProfileEventQuery &other) const
    {
        bool result = false;
        if (m_rank == other.m_rank &&
            m_serial_begin == other.m_serial_begin &&
            m_serial_end == other.m_serial_end) {
            result = true;
        }
        return result;
    }

    int ProfileEventQuery::rank(void) const
    {
        return m_rank;
    }

    size_t ProfileEventQuery::serial_begin(void) const
    {
        return m_serial_begin;
    }

    size_t ProfileEventQuery::serial_end(void) const
    {
        return m_serial_end;
    }

    class ProfileEventBufferImp : public ProfileEventBuffer
    {
        public:
            ProfileEventBufferImp() = default;
            virtual ~ProfileEventBufferImp() = default;
            void cpu_rank(const std::vector<int> &rank_map);
            std::vector<int> cpu_rank(void) const;
            int num_rank(void) const;
            size_t insert(const struct geopm_prof_message_s &prof_msg);
            void thread_progress(std::vector<double> per_cpu_progress);
            std::vector<double> thread_progress(void) const;
            size_t serial_begin(void) const;
            size_t serial_end(void) const;
            int epoch_begin(void) const;
            uint64_t hash_begin(void) const;
            uint64_t hint_begin(void) const;
            int epoch_count(const ProfileEventQuery &query) const;
            std::vector<struct geopm_time_s> epoch_times(const ProfileEventQuery &query) const;
            std::set<uint64_t> hash_set(const ProfileEventQuery &query) const;
            std::vector<std::pair<uint64_t, bool> > hash_series(const ProfileEventQuery &query) const;
            int hash_count(const ProfileEventQuery &query,
                           uint64_t hash) const;
            double hash_time(const ProfileEventQuery &query,
                             uint64_t hash) const;
            uint64_t current_hash(const ProfileEventQuery &query,
                                  uint64_t hash) const;
            uint64_t current_hint(const ProfileEventQuery &query,
                                  uint64_t hint) const;
            double current_progress(const ProfileEventQuery &query,
                                    double progress) const;
        private:
            std::vector<int> m_rank_map;
    };

    ProfileEventBuffer &profile_event_buffer(void)
    {
        static ProfileEventBufferImp instance;
        return instance;
    }

    std::vector<int> ProfileEventBufferImp::cpu_rank(void) const
    {
        return m_rank_map;
    }

    void ProfileEventBufferImp::cpu_rank(const std::vector<int> &rank_map)
    {
        m_rank_map = rank_map;
    }

    int ProfileEventBufferImp::num_rank(void) const
    {
        return 0;
    }

    size_t ProfileEventBufferImp::insert(const struct geopm_prof_message_s &prof_msg)
    {
        return 0;
    }

    void ProfileEventBufferImp::thread_progress(std::vector<double> per_cpu_progress)
    {

    }

    std::vector<double> ProfileEventBufferImp::thread_progress(void) const
    {
        std::vector<double> result;
        return result;
    }

    size_t ProfileEventBufferImp::serial_begin(void) const
    {
        return 0;
    }

    size_t ProfileEventBufferImp::serial_end(void) const
    {
        return 0;
    }

    int ProfileEventBufferImp::epoch_begin(void) const
    {
        return -1;
    }

    uint64_t ProfileEventBufferImp::hash_begin(void) const
    {
        return GEOPM_REGION_HASH_UNMARKED;
    }

    uint64_t ProfileEventBufferImp::hint_begin(void) const
    {
        return GEOPM_REGION_HINT_UNKNOWN;
    }

    int ProfileEventBufferImp::epoch_count(const ProfileEventQuery &query) const
    {
        return 0;
    }

    std::vector<struct geopm_time_s> ProfileEventBufferImp::epoch_times(const ProfileEventQuery &query) const
    {
        std::vector<struct geopm_time_s> result;
        return result;
    }

    std::set<uint64_t> ProfileEventBufferImp::hash_set(const ProfileEventQuery &query) const
    {
        std::set<uint64_t> result;
        return result;
    }

    std::vector<std::pair<uint64_t, bool> > ProfileEventBufferImp::hash_series(const ProfileEventQuery &query) const
    {
        std::vector<std::pair<uint64_t, bool> > result;
        return result;
    }

    int ProfileEventBufferImp::hash_count(const ProfileEventQuery &query,
                                          uint64_t hash) const
    {
        return 0;
    }

    double ProfileEventBufferImp::hash_time(const ProfileEventQuery &query,
                                            uint64_t hash) const
    {
        return 0.0;
    }

    uint64_t ProfileEventBufferImp::current_hash(const ProfileEventQuery &query,
                                                 uint64_t hash) const
    {
        return GEOPM_REGION_HASH_UNMARKED;
    }

    uint64_t ProfileEventBufferImp::current_hint(const ProfileEventQuery &query,
                                                 uint64_t hint) const
    {
        return GEOPM_REGION_HINT_UNKNOWN;
    }

    double ProfileEventBufferImp::current_progress(const ProfileEventQuery &query,
                                                   double progress) const
    {
        return 0.0;
    }

}

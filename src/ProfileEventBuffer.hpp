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

#ifndef PROFILEEVENTBUFFER_HPP_INCLUDE
#define PROFILEEVENTBUFFER_HPP_INCLUDE

namespace geopm
{
    class ProfileEventQuery;

    class ProfileEventBuffer
    {
        public:
            std::unique_ptr<ProfileEventBuffer> make_unique(int num_rank, size_t capacity);
            ProfileEventBuffer() = default;
            virtual ~ProfileEventBuffer() = default;
            virtual int num_rank(void) const = 0;
            virtual size_t insert(const struct geopm_prof_message_s &prof_msg) = 0;
            virtual size_t serial_begin(void) = 0;
            virtual size_t serial_end(void) = 0;
            virtual int epoch_begin(void) const = 0;
            virtual uint64_t hash_begin(void) const = 0;
            virtual uint64_t hint_begin(void) const = 0;
            virtual void update_epoch_count(const ProfileEventQuery &query,
                                            int &num_epoch) const = 0;
            virtual void update_epoch_time(const ProfileEventQuery &query,
                                           double &total_epoch_time) const = 0;
            virtual std::set<uint64_t> hash_set(const ProfileEventQuery &query) const = 0;
            virtual std::vector<std::pair<uint64_t, bool> > hash_series(const ProfileEventQuery &query) const = 0;
            virtual void update_hash_count(const ProfileEventQuery &query,
                                           uint64_t hash,
                                           size_t &num_exit) const = 0;
            virtual void update_hash_time(const ProfileEventQuery &query,
                                          uint64_t hash,
                                          double &runtime) const = 0;
            virtual void update_current_hash(const ProfileEventQuery &query,
                                             uint64_t &current_hash) const = 0;
            virtual void update_current_hint(const ProfileEventQuery &query,
                                             uint64_t current_hint) const = 0;
            virtual void update_current_progress(const ProfileEventQuery &query,
                                                 double &current_progress) const = 0;
    };

    class ProfileEventQuery
    {
        public:
            ProfileEventQuery(int rank, size_t serial_begin, size_t serial_end);
            virtual ProfileEventQuery() = default;
            int rank(void) const;
            size_t serial_begin(void) const;
            size_t serial_end(void) const;
            void update_serial(size_t serial_end);
        private:
            int m_rank;
            size_t m_serial_begin;
            size_t m_serial_end;
    };

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
}

#endif

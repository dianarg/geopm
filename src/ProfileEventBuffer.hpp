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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>
#include <set>

struct geopm_prof_message_s;

namespace geopm
{
    class ProfileEventQuery
    {
        public:
            /// @brief Construct a query object for ProfileEventBuffer.
            /// @param [in] rank Node local rank index to query.
            /// @param [in] serial_begin First serial number in range
            ///        to query (inclusive).
            /// @param [in] serial_end Last serial number in range to
            ///        query (exclusive)
            ProfileEventQuery(int rank, size_t serial_begin, size_t serial_end);
            /// @brief Default destructor.
            virtual ~ProfileEventQuery() = default;
            /// @brief Update the range to query: specified serial
            ///        number is new last serial number, and first
            ///        serial number is updated to privious value for
            ///        last serial number.
            /// @param serial_end New value for last serial number
            void update_serial(size_t serial_end);
            /// @brief Equality comparison operator for queries.
            /// @param [in] other Query to compare with.
            /// @return True if rank and current serial bounds match.
            bool operator == (const ProfileEventQuery &other) const;
            /// @brief Get the rank of the request.
            /// @return The node local rank.
            int rank(void) const;
            /// @brief Get the current value of the first serial
            ///        number in range to query.
            /// @return Begining of serial number range (inclusive).
            size_t serial_begin(void) const;
            /// @brief Get the current value of the first serial
            ///        number in range to query.
            /// @return Begining of serial number range (inclusive).
            size_t serial_end(void) const;
        private:
            int m_rank;
            size_t m_serial_begin;
            size_t m_serial_end;
    };

    class ProfileEventBuffer
    {
        public:
            /// @brief Default constructor for pure virtual base class.
            ProfileEventBuffer() = default;
            /// @brief Default destructor for pure virtual base class.
            virtual ~ProfileEventBuffer() = default;
            /// @brief Enable mapping from local rank to global rank.
            ///        Must be called prior to insert().
            virtual void cpu_rank(const std::vector<int> &rank_map) = 0;
            /// @brief Get the number of ranks per node.
            /// @return Number of ranks per node.
            virtual int num_rank(void) const = 0;
            /// @brief Insert a message from the application.
            /// @param [in] prof_msg A profile message sent from the
            ///        application.
            /// @return Serial number assigned to the event.
            virtual size_t insert(const struct geopm_prof_message_s &prof_msg) = 0;
            /// @brief Update with a sample of thread progress.
            /// @param [in] Vector indexed over linux logical CPU of
            ///        the progress of the thread affinnitized to that CPU.
            virtual void thread_progress(std::vector<double> per_cpu_progress) = 0;
            /// @brief Get latest sample of per thread progress.
            /// @return Vector indexed over linux logical CPU of the
            ///         progress of the thread affinnitized to that
            ///         CPU.
            virtual std::vector<double> thread_progress(void) = 0;
            /// @brief Oldest serial number of an event that is still
            ///        stored in the object.
            /// @return Serial number that can be used to create a
            ///         ProfileEventQuery.
            virtual size_t serial_begin(void) const = 0;
            /// @brief One larger than the most recent serial number
            ///        of an event stored in the object.
            /// @return Serial number that can be used to create the a
            ///         ProfileEventQuery, or can be passed to the
            ///         update() method on an existing
            ///         ProfileEventQuery.
            virtual size_t serial_end(void) const = 0;
            /// @brief Initial value for the epoch count.
            /// @return Value to initialize an epoch count to so that
            ///         when the return of epoch_count() is added it
            ///         tracks the total number of completed epochs
            ///         (i.e. -1).
            virtual int epoch_begin(void) const = 0;
            /// @brief Initial value for the region hash.
            /// @return GEOPM_REGION_HASH_UNMARKED.
            virtual uint64_t hash_begin(void) const = 0;
            /// @brief Initial value for the region hint.
            /// @return GEOPM_REGION_HINT_UNKNOWN
            virtual uint64_t hint_begin(void) const = 0;
            /// @brief Counts the number of epoch events within the
            ///        query.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @return Value to be added into a running count of
            ///         epoch events.
            virtual int epoch_count(const ProfileEventQuery &query) const = 0;
            /// @brief Get series of times when epochs occurred within
            ///        the query.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @return vector of epoch event times.
            virtual std::vector<struct geopm_time_s> epoch_times(const ProfileEventQuery &query) const = 0;
            /// @brief Get a set of all region hashes that were
            ///        entered or exited within the query.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @return Set of hashes that can be iterated over when
            ///         calling hash_count() and hash_time() with the
            ///         same query.
            virtual std::set<uint64_t> hash_set(const ProfileEventQuery &query) const = 0;
            /// @brief Get a series of region entry and exit events
            ///        within the query.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @return Vector of pairs where first element is the
            ///         region hash and second element is false for an
            ///         entry event and true for exit event.
            virtual std::vector<std::pair<uint64_t, bool> > hash_series(const ProfileEventQuery &query) const = 0;
            /// @brief Get a count of the number of region exit events
            ///        that occurred within the query for the region
            ///        with the specified hash.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @param [in] hash Hash for an observed region as is
            ///        returned by the hash_set() method.
            /// @return Value that can be added to a running count of
            ///         the number of completions of the specified
            ///         region.
            virtual int hash_count(const ProfileEventQuery &query,
                                   uint64_t hash) const = 0;
            /// @brief Get total runtime for all completed occurrences
            ///        of the region within the query.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @param [in] hash Hash for an observed region as is
            ///        returned by the hash_set() method.
            /// @return Cumulative amount of time executed with
            ///         occurrences of the specified region exited
            ///         within the query.
            virtual double hash_time(const ProfileEventQuery &query,
                                     uint64_t hash) const = 0;
            /// @brief Update the hash that is executing after the
            ///        last event within the query.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @param [in] hash Region that was executing at the time
            ///        of the sample just prior to the query
            ///        (i.e. value from last update).  Use
            ///        hash_begin() to initialize this value prior to
            ///        first call to current_hash().
            virtual uint64_t current_hash(const ProfileEventQuery &query,
                                          uint64_t hash) const = 0;
            /// @brief Update the hint that is current after the last
            ///        event within the query.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @param [in] hint Hint that describes application just
            ///        prior to the query.  (i.e. value from last
            ///        update).  Use hint_begin() to initialize this
            ///        value prior to first call to current_hint().
            virtual uint64_t current_hint(const ProfileEventQuery &query,
                                          uint64_t hint) const = 0;
            /// @brief Update the progress to reflect events within
            ///        the query.
            /// @param [in] query Specifies the rank and range of
            ///        serial numbers for the request.
            /// @param [in] progress Value of progress for the
            ///        currently executing region after last update.
            ///        Initialize to zero for first call.  This value
            ///        is returned if the region does not change and
            ///        no progress events occur within the query.
            /// @return Progress of within the region executing at the
            ///         end of the query.
            virtual double current_progress(const ProfileEventQuery &query,
                                            double progress) const = 0;
    };

    ProfileEventBuffer &profile_event_buffer(void);
}

#endif

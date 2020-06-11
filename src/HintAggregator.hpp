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

#ifndef HINTAGGREGATOR_HPP_INCLUDE
#define HINTAGGREGATOR_HPP_INCLUDE

#include <cstdint>

#include <map>

namespace geopm
{
    /// Aggregates time spent in each region hint, including
    /// GEOPM_REGION_HINT_UNKNOWN.  It is assumed that each object of
    /// this class will track one region or the epoch for a single
    /// process.  The caller should use the stream of records from
    /// ApplicationSampler to decide when to start, stop, or change
    /// the hint.
    class HintAggregator
    {
        public:
            HintAggregator();
            virtual ~HintAggregator() = default;
            /// @brief Begin collecting time from the given timestamp
            ///        with GEOPM_REGION_HINT_UNKNOWN as the default
            ///        hint.
            void start(double time);
            /// @brief Stop collecting time and add any remaining time
            ///        up to the given timestamp to the current hint.
            ///        This method should not be called without a
            ///        prior call to start().
            void stop(double time);
            /// @brief Update the time for the current hint and start
            ///        collecting time for the new hint.  This method
            ///        should not be called without a prior call to
            ///        start().
            void change_hint(uint64_t hint, double time);
            /// @brief Return the accumulated time for the given hint.
            ///        By default it returns 0.
            double get_total_runtime(uint64_t hint) const;
        private:
            std::map<uint64_t, double> m_hint_runtime;
            uint64_t m_current_hint;
            double m_start_time;
    };
}

#endif

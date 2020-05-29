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

#ifndef EPOCHRECORDFILTER_HPP_INCLUDE
#define EPOCHRECORDFILTER_HPP_INCLUDE

#include "RecordFilter.hpp"

namespace geopm
{
    /// @brief Filter to retain only records that are used in
    ///        caluculating signals for the EpochIOGroup.
    class EpochRecordFilter : public RecordFilter
    {
        public:
            /// @brief Default constructor.
            EpochRecordFilter() = default;
            /// @brief Default destructor.
            virtual ~EpochRecordFilter() = default;
            /// @brief The M_EVENT_EPOCH and M_EVENT_HINT event types
            ///        are retained, all other records are removed
            ///        from the stream.
            ///
            /// @param record The update value to be filtered.
            ///
            /// @return Vector of lenth one containing input value if
            ///         event type matches, otherwise returns empty
            ///         vector.
            std::vector<ApplicationSampler::m_record_s> filter(const ApplicationSampler::m_record_s &record);
    };
}

#endif

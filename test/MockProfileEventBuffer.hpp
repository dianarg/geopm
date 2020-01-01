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

#ifndef MOCKPROFILEEVENTBUFFER_HPP_INCLUDE
#define MOCKPROFILEEVENTBUFFER_HPP_INCLUDE

#include "gmock/gmock.h"

#include "ProfileEventBuffer.hpp"

class MockProfileEventBuffer : public geopm::ProfileEventBuffer
{
    public:
        MOCK_METHOD1(cpu_rank,
                     void(const std::vector<int> &rank_map));
        MOCK_CONST_METHOD0(cpu_rank,
                           std::vector<int>(void));
        MOCK_CONST_METHOD0(num_rank,
                           int(void));
        MOCK_METHOD1(insert,
                     size_t(const struct geopm_prof_message_s &prof_msg));
        MOCK_METHOD1(thread_progress,
                     void(std::vector<double> per_cpu_progress));
        MOCK_CONST_METHOD0(thread_progress,
                           std::vector<double>(void));
        MOCK_METHOD1(report_name,
                     void(const std::string &name));
        MOCK_CONST_METHOD0(report_name,
                           std::string(void));
        MOCK_METHOD1(profile_name,
                     void(const std::string &name));
        MOCK_CONST_METHOD0(profile_name,
                           std::string(void));
        MOCK_METHOD1(region_name_set,
                     void(const std::set<std::string> &name_set));
        MOCK_CONST_METHOD0(region_name_set,
                           std::set<std::string>(void));
        MOCK_CONST_METHOD0(serial_begin,
                           size_t(void));
        MOCK_CONST_METHOD0(serial_end,
                           size_t(void));
        MOCK_CONST_METHOD0(epoch_begin,
                           int(void));
        MOCK_CONST_METHOD0(hash_begin,
                           uint64_t(void));
        MOCK_CONST_METHOD0(hint_begin,
                           uint64_t(void));
        MOCK_CONST_METHOD1(epoch_count,
                           int(const geopm::ProfileEventQuery &query));
        MOCK_CONST_METHOD1(epoch_times,
                           std::vector<geopm_time_s>(const geopm::ProfileEventQuery &query));
        MOCK_CONST_METHOD1(hash_set,
                           std::set<uint64_t>(const geopm::ProfileEventQuery &query));
        MOCK_CONST_METHOD1(hash_series,
                           std::vector<std::pair<uint64_t, bool> >(const geopm::ProfileEventQuery &query));
        MOCK_CONST_METHOD2(hash_count,
                           int(const geopm::ProfileEventQuery &query, uint64_t hash));
        MOCK_CONST_METHOD2(hash_time,
                           double(const geopm::ProfileEventQuery &query, uint64_t hash));
        MOCK_CONST_METHOD2(current_hash,
                           uint64_t(const geopm::ProfileEventQuery &query, uint64_t hash));
        MOCK_CONST_METHOD2(current_hint,
                           uint64_t(const geopm::ProfileEventQuery &query, uint64_t hint));
        MOCK_CONST_METHOD2(current_progress,
                           double(const geopm::ProfileEventQuery &query, double progress));
};

#endif

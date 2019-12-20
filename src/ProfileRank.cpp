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

#include "geopm.h"
#include "ProfileRank.hpp"

namespace geopm
{
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

    uint64_t ProfileRank::region_hint(void)
    {
        return GEOPM_REGION_HINT_UNKNOWN;
    }

    int ProfileRank::region_count(void)
    {
        return 0;
    }

    double ProfileRank::region_runtime(void)
    {
        return 0.0;
    }

    double ProfileRank::region_progress(void)
    {
        return 0.0;
    }
}

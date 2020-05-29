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

#include "config.h"

#include <cmath>

#include "geopm.h"
#include "RecordProcessEpoch.hpp"

namespace geopm
{
    RecordProcessEpoch::RecordProcessEpoch()
        : m_epoch_count(-1)
        , m_last_epoch_time(NAN)
        , m_last_runtime(NAN)
        , m_curr_hint(GEOPM_REGION_HINT_UNKNOWN)
        , m_last_hint_time(NAN)
    {
        reset_hint_map(m_curr_hint_runtime, 0.0);
        reset_hint_map(m_last_hint_runtime, 0.0);
    }

    // todo: might want value input too
    void RecordProcessEpoch::reset_hint_map(std::map<uint64_t, double> &hint_map,
                                            double value)
    {
        hint_map[GEOPM_REGION_HINT_UNKNOWN] = value;
        hint_map[GEOPM_REGION_HINT_COMPUTE] = value;
        hint_map[GEOPM_REGION_HINT_MEMORY] = value;
        hint_map[GEOPM_REGION_HINT_NETWORK] = value;
        hint_map[GEOPM_REGION_HINT_IO] = value;
        hint_map[GEOPM_REGION_HINT_SERIAL] = value;
        hint_map[GEOPM_REGION_HINT_PARALLEL] = value;
        hint_map[GEOPM_REGION_HINT_IGNORE] = value;
    }

    void RecordProcessEpoch::update(const ApplicationSampler::m_record_s &record)
    {
        switch (record.event) {
            case ApplicationSampler::M_EVENT_EPOCH_COUNT:
                // update count
                m_epoch_count = record.signal;
                // update last runtime
                if (!std::isnan(m_last_epoch_time)) {
                    m_last_runtime = record.time - m_last_epoch_time;
                }
                m_last_epoch_time = record.time;

                // attribute current hint time to the previous epoch
                if (!std::isnan(m_last_hint_time)) {
                    m_curr_hint_runtime[m_curr_hint] += record.time - m_last_hint_time;
                }
                m_last_hint_time = record.time;
                // save off totals for all hints
                m_last_hint_runtime = m_curr_hint_runtime;
                reset_hint_map(m_curr_hint_runtime, 0.0);
                break;
            case ApplicationSampler::M_EVENT_HINT:
                // update total for previous hint
                if (!std::isnan(m_last_hint_time)) {
                    m_curr_hint_runtime[m_curr_hint] += record.time - m_last_hint_time;
                }
                // update current hint
                m_curr_hint = record.signal;
                m_last_hint_time = record.time;
                break;
            default:
                break;
        }

    }

    double RecordProcessEpoch::last_epoch_runtime(void) const
    {
        return m_last_runtime;
    }

    double RecordProcessEpoch::last_epoch_runtime_network(void) const
    {
        return m_last_hint_runtime.at(GEOPM_REGION_HINT_NETWORK);
    }

    double RecordProcessEpoch::last_epoch_runtime_ignore(void) const
    {
        return m_last_hint_runtime.at(GEOPM_REGION_HINT_IGNORE);
    }

    int RecordProcessEpoch::epoch_count(void) const
    {
        return m_epoch_count;
    }
}

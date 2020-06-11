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

#include "HintAggregator.hpp"

#include "geopm.h"
#include "Exception.hpp"

namespace geopm
{
    // std::unique_ptr<HintAggregator> HintAggregator::make_unique(void)
    // {
    //     return geopm::make_unique<HintAggregatorImp>();
    // }

    // std::shared_ptr<HintAggregator> HintAggregator::make_shared(void)
    // {
    //     return std::make_shared<HintAggregatorImp>();
    // }

    HintAggregator::HintAggregator()
        : m_current_hint(GEOPM_REGION_HINT_UNKNOWN)
        , m_start_time(NAN)
    {
        m_hint_runtime = {
            {GEOPM_REGION_HINT_UNKNOWN, 0},
            {GEOPM_REGION_HINT_COMPUTE, 0},
            {GEOPM_REGION_HINT_MEMORY, 0},
            {GEOPM_REGION_HINT_NETWORK, 0},
            {GEOPM_REGION_HINT_SERIAL, 0},
            {GEOPM_REGION_HINT_PARALLEL, 0},
            {GEOPM_REGION_HINT_IGNORE, 0},
        };
    }

    void HintAggregator::start(double time)
    {
        m_start_time = time;
        m_current_hint = GEOPM_REGION_HINT_UNKNOWN;
    }

    void HintAggregator::stop(double time)
    {
        if (std::isnan(m_start_time)) {
            throw Exception("HintAggregator::stop(): aggregator is not running.",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        double delta = time - m_start_time;
        m_hint_runtime.at(m_current_hint) += delta;
        m_start_time = NAN;
    }

    static void check_hint(uint64_t hint)
    {
        switch (hint) {
            case GEOPM_REGION_HINT_UNKNOWN:
            case GEOPM_REGION_HINT_COMPUTE:
            case GEOPM_REGION_HINT_MEMORY:
            case GEOPM_REGION_HINT_NETWORK:
            case GEOPM_REGION_HINT_SERIAL:
            case GEOPM_REGION_HINT_PARALLEL:
            case GEOPM_REGION_HINT_IGNORE:
                break;
            default:
                throw Exception("HintAggregator: invalid hint " + std::to_string(hint),
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    void HintAggregator::change_hint(uint64_t hint, double time)
    {
        if (std::isnan(m_start_time)) {
            throw Exception("HintAggregator::stop(): aggregator is not running.",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        check_hint(hint);
        double delta = time - m_start_time;
        m_hint_runtime.at(m_current_hint) += delta;
        m_start_time = time;
        m_current_hint = hint;
    }

    double HintAggregator::get_total_runtime(uint64_t hint) const
    {
        check_hint(hint);
        return m_hint_runtime.at(hint);
    }
}

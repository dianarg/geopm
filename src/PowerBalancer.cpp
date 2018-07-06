/*
 * Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
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

#include <vector>

#include "PowerBalancer.hpp"

namespace geopm
{
    PowerBalancer::PowerBalancer(double initial_budget)
        : M_STABLE_FACTOR(0.1)
        , m_is_stable(false)
        , m_step(M_STEP_MEASURE_RUNTIME)
        , m_iteration(0)
        , m_power_cap(initial_budget)
        , m_power_budget(initial_budget)
        , m_target_runtime(0.0)
    {

    }

    void PowerBalancer::runtime_update(double runtime)
    {
        if (m_step != M_STEP_MEASURE_RUNTIME) {
            throw Exception("PowerBalancer::runtime_update(): called when not in measure runtime step.")
        }
        m_runtime_buffer.insert(runtime);
    }

    bool PowerBalancer::runtime_stable(void)
    {
        if (m_step != M_STEP_MEASURE_RUNTIME &&
            m_step != M_STEP_REDUCE_BUDGET) {
            throw Exception("PowerBalancer::runtime_stable(): called when not in measure runtime or reduce budget step.",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        bool result = true;
        if (m_runtime_buffer.size() < 3) {
            result = false;
        }
        else {
            double runtime_avg = 0.0;
            double runtime_stddev = 0.0;
            runtime_stats(runtime_avg, runtime_stddev);
            if (runtime_stddev / runtime_avg > M_STABLE_FACTOR) {
                result = false;
            }
        }
        if (result) {
            ++m_step;
        }
        return result;
    }

    void PowerBalancer::runtime_stats(double &runtime_avg, double &runtime_stddev)
    {
        std::vector<double> runtime_vec = m_runtime_buffer.make_buffer();
        runtime_avg = IPlatformIO::agg_average(runtime_vec);
        runtime_stddev = IPlatformIO::agg_stddev(runtime_vec);
    }

    void PowerBalancer::runtime_target(double runtime)
    {
        if (m_step != M_STEP_SEND_DOWN_RUNTIME) {
            throw Exception("PowerBalancer::runtime_target(): ");
        }
        m_runtime_target = runtime;
        ++m_step;
    }

    double PowerBalancer::budget_trial(void)
    {
        if (runtime_stable()) {
            double runtime_avg = 0.0;
            double runtime_stddev = 0.0;
            runtime_stats(runtime_avg, runtime_stddev);
            if (runtime_avg - runtime_stddev < m_runtime_target) {
                m_power_budget -= m_power_click;
                m_runtime_buffer.clear();
            }
            else if (runtime_avg > m_runtime_target &&
                     m_power_budget + m_power_click <= m_power_cap) {
                m_power_budget += m_power_click;
                m_is_excess_ready = true;
            }
        }
        return m_power_budget;
    }
}


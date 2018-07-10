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
#include "config.h"

namespace geopm
{
    PowerBalancer::PowerBalancer()
        : M_TARGET_EPSILON(0.03)
        , M_TRIAL_DELTA(1.0)
        , M_NUM_SAMPLES(5)
        , m_is_stable(false)
        , m_is_excess_ready(false)
        , m_step(M_STEP_SEND_DOWN_LIMIT)
        , m_iteration(0)
        , m_power_cap(0.0)
        , m_power_limit(0.0)
        , m_target_runtime(0.0)
        , m_runtime_buffer(M_NUM_SAMPLES)
    {

    }

    void PowerBalancer::power_cap(double cap)
    {
        m_power_limit = cap;
        m_power_cap = cap;
        m_runtime_buffer.clear();
    }

    double PowerBalancer::power_cap(void)
    {
        return m_power_cap;
    }

    double PowerBalancer::power_limit(void)
    {
        if (m_power_limit == 0.0) {
            throw Exception("PowerBalancer::power_limit() called prior to power_cap().",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        return m_power_limit;
    }

    bool PowerBalancer::is_runtime_stable(double measured_runtime)
    {
        bool result = false;
        if (m_runtime_buffer.size() == m_runtime_buffer.capacity()) {
            result = true;
        }
        return result;
    }

    double PowerBalancer::runtime_sample(void)
    {
        return IPlatformIO::agg_median(m_runtime_buffer.make_vector())
    }

    void PowerBalancer::target_runtime(double largest_runtime)
    {
        m_target_runtime = largest_runtime;
    }

    bool PowerBalancer::is_target_met(double measured_runtime)
    {
        bool result = false;
        m_runtime_buffer.insert(mearured_runtime);
        if (m_target_runtime * (1.0 - M_TARGET_EPSILON) < runtime_sample()) {
            result = true;
        }
        else {
            m_power_limit -= M_TRIAL_DELTA;
        }

        return result;
    }
}

#if 0






    PowerBalancer::PowerBalancer(double initial_budget)
        : M_NUM_SAMPLES(10)
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

#endif

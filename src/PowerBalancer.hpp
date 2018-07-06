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

#ifndef POWERBALANCER_HPP_INCLUDE
#define POWERBALANCER_HPP_INCLUDE

namespace geopm
{
    class PowerBalancer
    {
        public:
            enum m_step_e {
                /// @brief Measure epoch runtime at least 3 times and
                ///        keep a running average.
                M_STEP_MEASURE_RUNTIME,
                /// @brief Aggregate epoch runtime up tree by applying
                ///        maximum filter to measured values.
                M_STEP_SEND_UP_RUNTIME,
                /// @brief Propegate down from root the longest
                ///        recorded runtime from any node.
                M_STEP_SEND_DOWN_RUNTIME,
                /// @brief Decrease power budget on all nodes (other
                ///        than the slowest) until epoch runtime
                ///        matches the slowest.
                M_STEP_REDUCE_BUDGET,
                /// @brief Aggregate amount power limit was reduced in
                ///        last step up the tree with sum filter.  (Go
                ///        to M_STEP_SEND_DOWN_BUDGET next).
                M_STEP_SEND_UP_EXCESS,
                /// @brief On first iteration send down resource
                ///        manager average budget requested, otherwise
                ///        send down average excess power.
                M_STEP_SEND_DOWN_BUDGET,
                /// @brief Increase power budget applied by delta sent
                ///        down to leaf in last step or set it to
                ///        average if first iteration.
                M_STEP_ADJUST_BUDGET,
                /// @brief Number of steps in process.
                M_NUM_STEP,
            }

            PowerBalancer(double initial_budget);
            virtual ~PowerBalancer() = default;
            void runtime_update(double runtime);
            bool runtime_stable(void);
            double runtime_sample(void);
            void runtime_target(double runtime);
            double budget_trial(void);
            bool budget_stable(void);
            double budget_excess(void);
            void budget_increase(double power_delta);
        private:
            const double M_STABLE_FACTOR;
            bool m_is_stable;
            bool m_is_excess_ready;
            int m_step;
            int m_iteration;
            double m_power_cap;
            double m_power_budget;
            double m_target_runtime;
            CircularBuffer<double> m_runtime_buffer;
    };
}

#endif

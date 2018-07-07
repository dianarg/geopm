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
    /// @brief Stay within a power cap but redistibute power to
    ///        optimize performance. An average per compute node power
    ///        maximum is maintained, but individual nodes will be
    ///        allowed above or below this average.
    class PowerBalancer
    {
        public:
            /// @brief Construct a PowerBalancer object.
            PowerBalancer() = default;
            /// @brief Destory a PowerBalancer object.
            virtual ~PowerBalancer() = default;
            /// @brief Should be called at the start of application
            ///        execution with the average power cap across
            ///        compute nodes. Should be called at the end of
            ///        the second phase of execution to note that the
            ///        power savings made across all compute nodes has
            ///        been evenly redistributed.
            /// @param power_cap The new maximum power limit equal to
            ///        the current power limit plus the amount of
            ///        power saved that is being redistributed.
            void power_cap(double cap);
            /// @brief The current power cap which cannot be exceeded
            ///        without breaking contract that the average
            ///        power budget across all compute nodes is
            ///        maintained.
            /// @return The current value of the power cap.
            double power_cap(void);
            /// @brief Set the current power limit prescribed for this
            ///        node.  Initially should be set to the power
            ///        limit that will be maintained on average across
            ///        the compute nodes allocated to an application.
            /// @param limit The maximum power consumption allowed for
            ///        the compute node being managed.
            void power_limit(double limit);
            /// @brief Returns the current power limit prescribed for
            ///        this node.
            double power_limit(void);
            /// @brief Update the object with a runtime measured under
            ///        the current power limit and test if the current
            ///        runtime sample is reliable and a call
            ///        runtime_sample() can be made.
            /// @param measured_runtime Most recent measurement of the
            ///        execution time for an epoch on the node being
            ///        managed under the current power limit.
            bool is_runtime_stable(double measured_runtime);
            /// @brief Sample the measured runtimes under the current
            ///        power limit.  In the first phase of execution
            ///        this measurement will aggreated across all
            ///        compute nodes to find the largest runtime
            ///        measured.  In the second phase of execution,
            ///        the value will be passed to next_limit() in
            ///        order to test the execution time at a lower
            ///        limit.
            /// @return The expected execution time of an application
            ///         epoch under the current power limit.
            double runtime_sample(void);
            /// @param Set the target runtime which is the largest
            ///        epoch execution time measured since the
            ///        application began or the last global increase
            ///        to the power budget.
            /// @param largest_runtime The largest expected runtime
            ///        for one epoch across all compute nodes under
            ///        the current power budget.
            void target_runtime(double largest_runtime);
            /// @brief During the second phase of execution the power
            ///        limit is increased until the epoch runtime on
            ///        the compute node under managment has increased
            ///        to the runtime of the slowest compute node.
            ///        This method is used to update the object with a
            ///        new measurement and also test if the current
            ///        power limit meets the requirements.  If the
            ///        method returns false, then the value returned
            ///        by power_limit() may be modified.  The new
            ///        power limit should enforced for the next epoch
            ///        execution.
            /// @param measured_runtime Most recent measurement of the
            ///        execution time for an epoch on the node being
            ///        managed under the current power limit.
            bool is_target_met(double measured_runtime);
        private:
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
                /// @brief Decrease power limit on all nodes (other
                ///        than the slowest) until epoch runtime
                ///        matches the slowest.
                M_STEP_REDUCE_LIMIT,
                /// @brief Aggregate amount power limit was reduced in
                ///        last step up the tree with sum filter.  (Go
                ///        to M_STEP_SEND_DOWN_LIMIT next).
                M_STEP_SEND_UP_EXCESS,
                /// @brief On first iteration send down resource
                ///        manager average limit requested, otherwise
                ///        send down average excess power.
                M_STEP_SEND_DOWN_LIMIT,
                /// @brief Increase power limit applied by delta sent
                ///        down to leaf in last step or set it to
                ///        average if first iteration.
                M_STEP_ADJUST_LIMIT,
                /// @brief Number of steps in process.
                M_NUM_STEP,
            };

            const int M_NUM_SAMPLES;
            bool m_is_stable;
            bool m_is_excess_ready;
            int m_step;
            int m_iteration;
            double m_power_cap; // Maximum power as set in last global budget increase.
            double m_power_limit; // Current power limit to get to target runtime which may be lower than the cap.
            double m_target_runtime;
            CircularBuffer<double> m_runtime_buffer;
    };
}

#endif

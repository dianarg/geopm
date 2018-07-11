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

#ifndef BALANCINGAGENT_HPP_INCLUDE
#define BALANCINGAGENT_HPP_INCLUDE

#include <vector>

#include "Agent.hpp"

namespace geopm
{
    class IPlatformIO;
    class IPlatformTopo;
    template <class type>
    class ICircularBuffer;
    class PowerGovernor;

    class PowerBalancerAgent : public Agent
    {
        public:
            enum m_step_e {
                /// @brief Measure epoch runtime several times and
                ///        apply median filter.
                M_STEP_MEASURE_RUNTIME,
                /// @brief Aggregate epoch runtime up tree by applying
                ///        maximum filter to measured values.
                M_STEP_SEND_UP_RUNTIME,
                /// @brief Propagate down from root the longest
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

            enum m_policy_e {
                /// @brief Step that the root is providing a policy
                ///        for.  The parent has recieved a sample
                ///        matching this step in the last walk up the
                ///        tree, execpt in the case where the endpoint
                ///        has recently been updated with a new
                ///        policy, in this case the step will be
                ///        M_SEND_DOWN_LIMIT and the policy indexed by
                ///        M_POLICY_POWER_CAP will have a non-zero
                ///        value.
                M_POLICY_STEP,
                /// @brief The power cap enforced on average over all
                ///        nodes running the application.  This has
                ///        value 0.0 except in two cases.  In the
                ///        first case this is the M_SEND_DOWN_LIMIT
                ///        step at the beginning of the application
                ///        run.  This value will also be non-zero in
                ///        the case where the resource mananger has
                ///        requested a new budget for the application,
                ///        and thus, the algorithm must be restarted
                ///        at step M_SEND_DOWN_LIMIT.
                M_POLICY_POWER_CAP,
                /// @brief The largest runtime reported by any leaf
                ///        agent since the last redistirubition of
                ///        power.  This will have value 0.0 until all
                ///        leaf agents have reported a runtime to the
                ///        root agent.
                M_POLICY_RUNTIME,
                /// @brief This value is updated in step
                ///        M_STEP_ADJUST_LIMIT to the amount that each
                ///        leaf agent should increase their power
                ///        limit by calling:
                ///            power_cap(current_limit + slack)
                ///        by before starting the algorithm over again
                ///        at step M_STEP_MEASURE_RUNTIME.  For all
                ///        other steps this value is 0.0.
                M_POLICY_POWER_SLACK,
                /// @brief Number of steps in each iteration of the
                ///        balancing algorithm.
                M_NUM_POLICY,
            };

            enum m_sample_e { // Tree samples
                M_SAMPLE_STEP,
                M_SAMPLE_IS_STEP_COMPLETE,
                M_SAMPLE_RUNTIME,
                M_SAMPLE_POWER_SLACK,
                M_NUM_SAMPLE,
            };

            enum m_plat_signal_e {
                M_PLAT_SIGNAL_EPOCH_RUNTIME,
                //M_PLAT_SIGNAL_EPOCH_ENERGY,
                M_PLAT_SIGNAL_EPOCH_COUNT,
                M_PLAT_SIGNAL_PKG_POWER,
                M_PLAT_SIGNAL_DRAM_POWER,
                M_PLAT_NUM_SIGNAL,
            };
            enum m_trace_sample_e {
                M_TRACE_SAMPLE_EPOCH_RUNTIME,
                M_TRACE_SAMPLE_PKG_POWER,
                M_TRACE_SAMPLE_DRAM_POWER,
                M_TRACE_SAMPLE_IS_CONVERGED,
                M_TRACE_SAMPLE_PWR_BUDGET,
                M_TRACE_NUM_SAMPLE,
            };

            PowerBalancerAgent();
            virtual ~PowerBalancerAgent();
            void init(int level, const std::vector<int> &fan_in, bool is_level_root) override;
            bool descend(const std::vector<double> &in_policy,
                         std::vector<std::vector<double> >&out_policy) override;
            bool ascend(const std::vector<std::vector<double> > &in_sample,
                        std::vector<double> &out_sample) override;
            bool adjust_platform(const std::vector<double> &in_policy) override;
            bool sample_platform(std::vector<double> &out_sample) override;
            void wait(void) override;
            std::vector<std::pair<std::string, std::string> > report_header(void) const override;
            std::vector<std::pair<std::string, std::string> > report_node(void) const override;
            std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > report_region(void) const override;
            std::vector<std::string> trace_names(void) const override;
            void trace_values(std::vector<double> &values) override;
            static std::string plugin_name(void);
            static std::unique_ptr<Agent> make_plugin(void);
            static std::vector<std::string> policy_names(void);
            static std::vector<std::string> sample_names(void);
        private:
            void init_platform_io(void);
            bool descend_initial_budget(double power_budget_in, std::vector<double> &power_budget_out);
            bool descend_updated_budget(double power_budget_in, std::vector<double> &power_budget_out);
            bool descend_updated_runtimes(double power_budget_in, std::vector<double> &power_budget_out);
            std::vector<double> split_budget(double avg_power_budget);
            std::vector<double> split_budget_first(double power_budget_in);
            std::vector<double> split_budget_helper(double avg_power_budget,
                                                    double min_power_budget,
                                                    double max_power_budget);

            IPlatformIO &m_platform_io;
            IPlatformTopo &m_platform_topo;
            int m_level; // Needed in order to determine convergence
            bool m_is_converged;
            bool m_is_sample_stable;
            int m_updates_per_sample;
            int m_samples_per_control;
            double m_min_power_budget;
            double m_max_power_budget;
            std::unique_ptr<PowerGovernor> m_power_gov;
            std::unique_ptr<IPowerBalancer> m_power_balancer;
            std::vector<int> m_pio_idx;
            std::vector<std::function<double(const std::vector<double>&)> > m_agg_func;
            int m_num_children;
            bool m_is_root;
            double m_last_power_budget_in;
            double m_last_power_budget_out;
            std::vector<double> m_last_runtime0;
            std::vector<double> m_last_runtime1;
            std::vector<double> m_last_budget0;
            std::vector<double> m_last_budget1;
            std::unique_ptr<ICircularBuffer<double> > m_epoch_runtime_buf;
            std::unique_ptr<ICircularBuffer<double> > m_epoch_power_buf;
            std::vector<double> m_sample;
            double m_last_energy_status;
            int m_sample_count;
            int m_ascend_count;
            const int m_ascend_period;
            bool m_is_updated;
            const double m_convergence_target;
            int m_num_out_of_range;
            const int m_min_num_converged;
            int m_num_converged;
            int m_last_epoch_count;
    };
}

#endif

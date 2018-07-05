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

#ifndef POWERCLICK_HPP_INCLUDE
#define POWERCLICK_HPP_INCLUDE

#include <vector>
#include <memory>

namespace geopm
{
    class IPowerClickBuffer;

    class IPowerClick
    {
        public:
            IPowerClick() = default;
            virtual ~IPowerClick() = default;
            /// @brief Update power settings given the runtime
            ///        recorded for current settings.
            /// @param [in] child_idx Index of child reporting new
            ///        runtime measurement.
            /// @param [in] runtime Last runtime recorded by specified
            ///        child under the current policy.
            /// @return True if power policy has been changed, false
            ///         if the power policy was not changed.
            virtual bool update_runtime(int child_idx, double runtime) = 0;
            /// @brief The current power policy.
            /// @return Vector containing the power limit for each
            ///         child.
            virtual std::vector<double> policy(void) const = 0;
            /// @brief Double the number of discrete settings
            ///        ("clicks") used.
            /// @return True if the resolution was succesfully
            ///         refined, and false if the refinement would
            ///         require a granularity of control that is too
            ///         fine.
            virtual bool refine_delta(void) = 0;
            /// @brief The current value of one click.
            /// @return Current increment of one click in units of
            ///         Watts.
            virtual double power_delta(void) const = 0;
            /// @brief Minimum power setting.
            /// @return The lowest power setting allowed in units of
            ///         Watts.
            virtual double power_min(void) const = 0;
            /// @brief Maximum power setting.
            /// @return The highest power setting allowed in units of
            ///         Watts.
            virtual double power_max(void) const = 0;
    };

    class PowerClick : public IPowerClick
    {
        public:
            PowerClick(double power_init,
                       double power_min,
                       double power_max,
                       double min_delta,
                       int num_children,
                       int num_click);
            PowerClick(double power_init,
                       double power_min,
                       double power_max,
                       double min_delta,
                       int num_children,
                       int num_click,
                       std::vector<std::shared_ptr<IPowerClickBuffer> > mock_runtime_buf);
            virtual ~PowerClick() = default;
            bool update_runtime(int child_idx, double runtime) override;
            std::vector<double> policy(void) const override;
            bool refine_delta(void) override;
            double power_delta(void) const override;
            double power_min(void) const override;
            double power_max(void) const override;

            // Below here are internal methods that can be tested that
            // are used to implement update_runtime() and policy().

            /// @brief Modify the click_policy based on the runtime measurements.
            /// @param [in] runtime Current best runtime estimate in
            ///        units of seconds.
            /// @param [in] runtime_stddev Current estimate of error
            ///        window for runtime value in units of seconds.
            ///        Note that a value of zero is interpreted as not
            ///        enough samples to evaluate error.
            /// @param [out] click_policy Best estimate of the power
            ///        limits for each child in units of clicks.
            bool update_runtime(const std::vector<double> &runtime,
                                const std::vector<double> &runtime_stddev,
                                std::vector<int> &click_policy) const;
            /// @brief Convert a policy of clicks into a policy in Watts.
            /// @param click_policy Vector of policies in click units.
            /// @return Vector of policies converted into units of Watts.
            std::vector<double> policy(const std::vector<int> &click_policy) const;
        private:
            const double m_power_min;
            const double m_power_max;
            const double m_min_delta;
            int m_num_click;
            double m_delta;
            /// @brief current policy in units of clicks.
            std::vector<int> m_click_policy;
            /// @brief vector over children runtime data collected.
            std::vector<std::shared_ptr<IPowerClickBuffer> > m_runtime_buf;
    };
}

#endif

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

namespace geopm
{
    class IPowerClickBuffer;

    class PowerClick
    {
        public:
            PowerClick(double power_init,
                       double power_min,
                       double power_max,
                       double min_delta,
                       int num_children,
                       int num_click);
            virtual ~PowerClick() = default;
            /// @brief Update power settings given the runtime
            ///        recorded for current settings.
            /// @param child_idx Index of child reporting new runtime
            ///        measurement.
            /// @param runtime Last runtime recorded by specified child
            ///        under the current policy.
            /// @return True if power policy has been changed, false
            ///         if the power policy was not changed.
            bool update_runtime(int child_idx, double runtime);
            /// @brief The current power policy.
            /// @return Vector containing the power limit for each
            ///         child.
            std::vector<double> policy(void) const;
            /// @brief Double the number of discrete settings
            ///        ("clicks") used.
            bool refine_delta(void);

            // Below here are internal methods that can be tested
            // that are used to implement methods above.

            /// @brief Modify the click_policy based on the runtime measurements. This
            /// @return True if the resolution was succesfully
            ///         refined, and false if the refinement would
            ///         require a granularity of control that is too
            ///         fine.
            bool update_runtime(const std::vector<double> &runtime,
                                const std::vector<double> &runtime_stddev,
                                std::vector<int> &click_policy) const;
            /// @brief The current value of one click.
            /// @return Current increment of one click in units of
            ///         Watts.
            double power_delta(void) const;
            double power_min(void) const;
            double power_max(void) const;
            /// @brief Convert a policy of clicks to a policy in Watts.
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
            std::vector<std::unique_ptr<IPowerClickBuffer> > m_runtime_buf;
    };

}

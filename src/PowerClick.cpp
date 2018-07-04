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
    PowerClick::PowerClick(double power_min,
                           double power_max,
                           double power_init,
                           double min_delta,
                           int num_children,
                           int num_click)
        : m_power_min(power_min)
        , m_power_max(power_max)
        , m_min_delta(min_delta)
        , m_num_click(num_click)
    {
        if (num_click <= 1 ||
            min_power >= max_power ||
            power_init < min_power ||
            power_init > max_power ||
            num_children < 2) {
            throw Exception("PowerClick::PowerClick()",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        m_delta = power_delta();
        int click_init = (power_init - power_min) / m_delta;
        m_click_policy.resize(num_children, click_init);
    }

    double PowerClick::power_delta(void)
    {
        return (m_max_power - m_min_power) / (m_num_click - 1);
    }

    void PowerClick::click_to_power(const std::vector<int> &click,
                                    std::vector<double> &power)
    {
        if (click.size() != power.size()) {
            throw Exception("PowerClick::click_to_power(): input vectors not of equal size",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        auto power_it = power.begin();
        for (const auto &click_it : click) {
            *power_it = m_min_power + click_it * m_delta;
            ++power_it;
        }
    }

    bool PowerClick::refine(void)
    {
        bool result = true;
        int init_num_click = m_num_click;

        m_num_click *= 2;
        if (power_delta() >= m_min_delta) {
            m_delta = power_delta();
            for (auto &it : m_click_policy) {
                it *= 2;
            }
        }
        else {
            m_num_click = init_num_click;
            result = false;
        }
        return result;
    }

    bool PowerClick::update(const std::vector<double> &runtime,
                            std::vector<double> &power_policy)
    {
        bool result = false;
        int slow_idx = -1;
        int fast_idx = -1;
        double slow_runtime = DBL_MAX;
        double fast_runtime = -1 * DBL_MAX;
        for (int child_idx = 1; child_idx != m_num_children; ++child_idx) {
            if (slow_runtime > runtime[child_idx] &&
                m_click_policy[child_idx] != m_max_click - 1) {
                slow_runtime = runtime[child_idx];
                slow_idx = child_idx;
            }
            else if (fast_runtime < runtime[child_idx] &&
                     m_click_policy[child_idx] != 0) {
                fast_runtime = runtime[child_idx];
                fast_idx = child_idx;
            }
        }
        if (slow_idx != -1 && fast_idx != -1) {
            m_click_policy[slow_idx]++;
            m_click_policy[fast_idx]--;
            result = true;
        }
        click_to_power(m_click_policy, power_policy);
        return result;
    }
}

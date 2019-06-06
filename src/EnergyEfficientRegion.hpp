/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
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

#ifndef ENERGYEFFICIENTREGION_HPP_INCLUDE
#define ENERGYEFFICIENTREGION_HPP_INCLUDE

#include <cmath>
#include <set>
#include <vector>
#include <memory>

#include "CircularBuffer.hpp"

namespace geopm
{
    /// @brief Holds the performance history of a Region.
    class EnergyEfficientRegion
    {
        public:
            virtual ~EnergyEfficientRegion() = default;
            virtual double freq(void) const = 0;
            virtual void update_freq_range(double freq_min, double freq_max, double freq_step) = 0;
            virtual void sample(double curr_perf_metric) = 0;
            virtual void update_exit() = 0;
            virtual void disable(void) = 0;
            virtual bool is_learning(void) const = 0;
    };

    class EnergyEfficientRegionImp : public EnergyEfficientRegion
    {
        public:
            EnergyEfficientRegionImp(double freq_min, double freq_max, double freq_step);
            virtual ~EnergyEfficientRegionImp() = default;
            double freq(void) const override;
            void update_freq_range(double freq_min, double freq_max, double freq_step) override;
            void sample(double curr_perf_metric) override;
            void update_exit() override;
            void disable(void);
            bool is_learning(void) const override;
        private:
            static const int M_MIN_PERF_SAMPLE = 5;//@todo make this dependent on number of freq domains, or even better ranks?
            struct FreqContext {
                FreqContext()
                    : num_increase(0)
                    , perf(M_MIN_PERF_SAMPLE)
                {
                };
                virtual ~FreqContext() = default;
                size_t num_increase;
                CircularBuffer<double> perf;
            };
            const double M_PERF_MARGIN;
            const size_t M_MAX_INCREASE;
            bool m_is_learning;
            uint64_t m_max_step;
            double m_freq_step;
            int m_curr_step;
            double m_freq_min;
            double m_target;
            std::vector<std::unique_ptr<FreqContext> > m_freq_ctx;
    };

} // namespace geopm

#endif

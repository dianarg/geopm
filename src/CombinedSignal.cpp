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

#include <cmath>

#include <algorithm>

#include "CombinedSignal.hpp"
#include "Exception.hpp"
#include "config.h"

namespace geopm
{

    double CombinedSignal::sample(std::vector<double> values) {
        return std::accumulate(values.begin(), values.end(), 0.0);
    }

    double PerRegionDerivativeCombinedSignal::sample(std::vector<double> values)
    {
#ifdef GEOPM_DEBUG
        // caller is expected to pass in vector of (region id, time, value).
        if (values.size() != 3) {
            throw Exception("PerRegionDerivativeCombinedSignal::sample(): expected 3 values.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        double region_id = values[0];
        if (m_time_buffer.find(region_id) == m_time_buffer.end()) {
            //m_time_buffer[region_id] = CircularBuffer<m_sample_s>(M_NUM_SAMPLE_HISTORY);
        }
        if (m_signal_buffer.find(region_id) == m_signal_buffer.end()) {
            //m_signal_buffer[region_id] = CircularBuffer<m_sample_s>(M_NUM_SAMPLE_HISTORY);
        }
        if (m_derivative_last.find(region_id) == m_derivative_last.end()) {
            m_derivative_last[region_id] = NAN;
        }
        /*
        double ins_time = values[1];
        double ins_signal = values[2];
        // insert time and signal
        m_time_buffer.insert(ins_time);
        m_signal_buffer.insert(ins_signal);

        // Least squares linear regression to approximate the
        // derivative with noisy data.
        double result = m_derivative_last.at(region_id);
        if (m_derivative_num_fit >= 2) {
            size_t buf_size = m_time_buffer->size();
            double A = 0.0, B = 0.0, C = 0.0, D = 0.0;
            double E = 1.0 / m_derivative_num_fit;
            double time_0 = m_time_buffer.at(region_id).value(buf_size - m_derivative_num_fit);
            const double sig_0 = m_signal_buffer.at(region_id).value(buf_size - m_derivative_num_fit);
            for (size_t buf_off = buf_size - m_derivative_num_fit;
                 buf_off < buf_size; ++buf_off) {
                double tt = m_time_buffer.at(region_id).value(buf_off);
                double time = tt - time_0; //geopm_time_diff(&time_0, &tt);
                double sig = m_signal_buffer.at(region_id).value(buf_off) - sig_0;
                A += time * sig;
                B += time;
                C += sig;
                D += time * time;
            }
            double ssxx = D - B * B * E;
            double ssxy = A - B * C * E;
            result = ssxy / ssxx;
            m_derivative_last[region_id] = result;
        }
        return result ? result : NAN;
        */
        return NAN;
    }
}

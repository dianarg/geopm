/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
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

#include "DerivativeSignal.hpp"

#include <cmath>
#include <unistd.h>

#include "Helper.hpp"

namespace geopm
{
    DerivativeSignal::DerivativeSignal(std::shared_ptr<Signal> time_sig,
                                       std::shared_ptr<Signal> y_sig,
                                       int num_sample_history,
                                       double sleep_time)
        : m_time_sig(time_sig)
        , m_y_sig(y_sig)
        , M_NUM_SAMPLE_HISTORY(num_sample_history)
        , m_history(M_NUM_SAMPLE_HISTORY)
        , m_derivative_num_fit(0)
        , m_batch_ready(false)
        , m_sleep_time(sleep_time)
    {

    }

    int DerivativeSignal::domain(void) const
    {
        return -1;
    }

    std::function<std::string(double)> DerivativeSignal::format_function(void) const
    {
        return string_format_hex;
    }

    void DerivativeSignal::setup_batch(void)
    {
        if (!m_batch_ready) {
            m_time_sig->setup_batch();
            m_y_sig->setup_batch();
            m_batch_ready = true;
        }
    }

    double DerivativeSignal::compute_next(CircularBuffer<m_sample_s> &history,
                                          int &num_fit,
                                          double time, double signal)
    {
        // insert time and signal
        history.insert({time, signal});
        if (num_fit < history.capacity()) {
            ++num_fit;
        }

        // Least squares linear regression to approximate the
        // derivative with noisy data.
        double result = NAN;
        if (num_fit >= 2) {
            size_t buf_size = history.size();
            double A = 0.0, B = 0.0, C = 0.0, D = 0.0;
            double E = 1.0 / num_fit;
            double time_0 = history.value(buf_size - num_fit).time;
            const double sig_0 = history.value(buf_size - num_fit).sample;
            for (size_t buf_off = buf_size - num_fit;
                 buf_off < buf_size; ++buf_off) {
                double tt = history.value(buf_off).time;
                double time = tt - time_0;
                double sig = history.value(buf_off).sample - sig_0;
                A += time * sig;
                B += time;
                C += sig;
                D += time * time;
            }
            double ssxx = D - B * B * E;
            double ssxy = A - B * C * E;
            result = ssxy / ssxx;
        }
        return result;
    }

    double DerivativeSignal::sample(void)
    {
        // todo: check that setupbatch has been called
        double time = m_time_sig->sample();
        double signal = m_y_sig->sample();
        return compute_next(m_history, m_derivative_num_fit, time, signal);
    }

    double DerivativeSignal::read(void)
    {
        double result = NAN;
        CircularBuffer<m_sample_s> temp_history(M_NUM_SAMPLE_HISTORY);
        int num_fit = 0;
        for (int ii = 0; ii < M_NUM_SAMPLE_HISTORY; ++ii) {
            double time = m_time_sig->read();
            double signal = m_y_sig->read();
            result = compute_next(temp_history, num_fit, time, signal);
            if (ii < M_NUM_SAMPLE_HISTORY) {
                usleep(m_sleep_time);            }

        }
        return result;
    }

}

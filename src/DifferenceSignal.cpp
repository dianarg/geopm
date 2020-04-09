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

#include "DifferenceSignal.hpp"

namespace geopm
{
    DifferenceSignal::DifferenceSignal(std::shared_ptr<Signal> minuend,
                                       std::shared_ptr<Signal> subtrahend)
        : m_minuend(minuend)
        , m_subtrahend(subtrahend)
    {
        // todo: check for null

        // todo: check consistent domain and units
        if (m_minuend->domain() != m_subtrahend->domain()) {

        }
    }

    int DifferenceSignal::domain(void) const
    {
        return m_minuend->domain();
    }

    std::function<std::string(double)> DifferenceSignal::format_function(void) const
    {
        return m_minuend->format_function();
    }

    void DifferenceSignal::setup_batch(void)
    {
        m_minuend->setup_batch();
        m_subtrahend->setup_batch();
    }

    double DifferenceSignal::sample(void)
    {
        // todo: check setupbatch
        return m_minuend->sample() - m_subtrahend->sample();
    }

    double DifferenceSignal::read(void)
    {
        return m_minuend->read() - m_subtrahend->read();
    }


}

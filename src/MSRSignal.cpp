/*
 * Copyright (c) 2015, 2016, 2017, Intel Corporation
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

#include "Signal.hpp"

namespace geopm
{
    /* NEEDS TO BE FIXED */
    Signal::Signal(int msr_size)
        : m_value(0.0)
        , m_lshift(0)
        , m_rshift(0)
        , m_mask(0xFFFFFFFFFFFFFFFF)
        , m_multiplier(1.0)
        , m_msr_size(msr_size)
        , m_raw_value_last(0)
        , m_msr_overflow_offset(0)
    {

    }

    Signal::Signal(int msr_size, int lshift, int rshift, uint64_t mask, double multiplier)
        : m_value(0.0)
        , m_lshift(lshift)
        , m_rshift(rshift)
        , m_mask(mask)
        , m_multiplier(multiplier)
        , m_msr_size(msr_size)
        , m_raw_value_last(0)
        , m_msr_overflow_offset(0)
    {

    }

    Signal::~Signal()
    {

    }

    double Signal::value(void)
    {
        return m_value;
    }

    void Signal::raw_value(uint64_t msr_val)
    {
       // Mask off bits beyond msr_size
        msr_value &= ((~0ULL) >> (64 - m_msr_size));
        // Deal with register overflow
        if (msr_value < m_raw_value_last) {
            m_msr_overflow_offset += pow(2, m_msr_size);
        }
        m_raw_value_last = msr_value;
        msr_value += m_msr_overflow_offset;
        m_value = (double)((((msr_value << m_lshift) >> m_rshift) & m_mask) * m_multiplier);
    }

    void Signal::msr_size(int size)
    {
        m_msr_size = msr_size;
    }

    void Signal::left_shift(int shift_size)
    {
        m_lshift = shift_size;
    }

    void Signal::right_shift(int shift_size)
    {
        m_rshift = shift_size;
    }

    void Signal::mask(uint64_t bitmask)
    {
        m_mask = bitmask;
    }

    void Signal::multiplier(double factor)
    {
        m_multiplier = factor;
    }

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

#include "config.h"
#include "SSTControl.hpp"

#include "SSTIO.hpp"

namespace geopm
{
    SSTControl::SSTControl(std::shared_ptr<SSTIO> sstio,
                           int cpu_idx, uint32_t command, uint32_t subcommand,
                           uint32_t interface_parameter, uint32_t write_value,
                           int begin_bit, int end_bit)
        : m_sstio(sstio)
        , m_cpu_idx(cpu_idx)
        , m_command(command)
        , m_subcommand(subcommand)
        , m_interface_parameter(interface_parameter)
        , m_write_value(write_value)
        , m_shift(begin_bit)
        , m_num_bit(end_bit - begin_bit + 1)
        , m_mask(((1ULL << m_num_bit) - 1) << begin_bit)
    {

    }

    void SSTControl::setup_batch(void)
    {
        // If (mbox or mmio ) ...
        m_adjust_idx = m_sstio->add_mbox_write(m_cpu_idx, m_command, m_subcommand,
                                               m_interface_parameter, m_write_value);

    }

    void SSTControl::adjust(double value)
    {
        // TODO: check if value in range of uint32_t
        //     : Or check if in range of mask
        m_sstio->adjust(m_adjust_idx, (uint32_t)value << m_shift);
    }

    void SSTControl::write(double value)
    {

    }

    void SSTControl::save(void)
    {

    }

    void SSTControl::restore(void)
    {

    }
}

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

#include "RawMSRSignal.hpp"

#include "geopm_hash.h"
#include "Exception.hpp"
#include "Helper.hpp"


#ifdef GEOPM_DEBUG
/// Used to check for errors that should never occur unless there is a
/// mistake in internal logic.  These checks will be removed in
/// release builds.
#define GEOPM_DEBUG_ASSERT(condition, fail_message)                             \
    if (!(condition)) {                                                         \
        throw Exception(std::string(__func__) + ": " + fail_message,            \
                        GEOPM_ERROR_LOGIC, __FILE__, __LINE__);                 \
    }
#else
#define GEOPM_DEBUG_ASSERT(condition, fail_message)
#endif


namespace geopm
{
    RawMSRSignal::RawMSRSignal(std::shared_ptr<MSRIO> msrio,
                               int cpu,
                               uint64_t offset,
                               int domain_type)
        : m_msrio(msrio)
        , m_cpu(cpu)
        , m_offset(offset)
        , m_domain_type(domain_type)
        , m_data(nullptr)
    {
        GEOPM_DEBUG_ASSERT(m_msrio != nullptr, "no valid MSRIO object.");
    }

    int RawMSRSignal::domain(void) const
    {
        return m_domain_type;
    }

    std::string RawMSRSignal::units(void) const
    {
        // raw MSRs have no units
        return "";
    }

    std::function<std::string(double)> RawMSRSignal::format_function(void) const
    {
        return string_format_raw64;
    }

    void RawMSRSignal::setup_batch(void)
    {
        GEOPM_DEBUG_ASSERT(m_msrio != nullptr, "no valid MSRIO object.");

        m_data = m_msrio->add_read(m_cpu, m_offset);

        GEOPM_DEBUG_ASSERT(m_data, "no memory mapped for signal value.");
    }

    double RawMSRSignal::sample(void)
    {
        GEOPM_DEBUG_ASSERT(m_data != nullptr, "no memory mapped for signal value.");

        // convert to double
        return geopm_field_to_signal(*m_data);
    }

    double RawMSRSignal::read(void)
    {
        GEOPM_DEBUG_ASSERT(m_msrio != nullptr, "no valid MSRIO object.");

        // convert to double
        return geopm_field_to_signal(m_msrio->read_msr(m_cpu, m_offset));
    }
}

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

#include "MSRFieldSignal.hpp"

#include "geopm_hash.h"
#include "Exception.hpp"
#include "Helper.hpp"
#include "MSR.hpp"  // for enums

//@todo: needs config.h; where to put this?
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
    MSRFieldSignal::MSRFieldSignal(std::shared_ptr<Signal> raw_msr,
                                   int begin_bit,
                                   int end_bit,
                                   int function,
                                   double scalar,
                                   int units,
                                   std::function<std::string(double)> format_func)
        : m_raw_msr(raw_msr)
        , m_function(function)
        , m_shift(begin_bit)
        , m_num_bit(end_bit - begin_bit + 1)
        , m_mask(((1ULL << m_num_bit) - 1) << begin_bit)
        , m_subfield_max((1ULL << m_num_bit) - 1)
        , m_scalar(scalar)
        , m_last_field(0)
        , m_num_overflow(0)
        , m_units(units)
        , m_format_func(format_func)
    {

        // todo: macro
        //#ifdef GEOPM_DEBUG
        // if (m_num_bit >= 64) {
        //     throw Exception("64 bit fields are not supported.",
        //                     GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        // }/
        //#endif

        GEOPM_DEBUG_ASSERT(raw_msr != nullptr,
                           "no valid raw Signal");
        GEOPM_DEBUG_ASSERT(m_num_bit < 64, "64-bit fields are not supported");
        GEOPM_DEBUG_ASSERT(begin_bit <= end_bit,
                           "begin bit must be <= end bit");
        GEOPM_DEBUG_ASSERT(m_function >= MSR::M_FUNCTION_SCALE &&
                           m_function <= MSR::M_FUNCTION_OVERFLOW,
                           "invalid encoding function");
    }

    int MSRFieldSignal::domain(void) const
    {
        return m_raw_msr->domain();
    }

    std::string MSRFieldSignal::units(void) const
    {
        std::string result = "";
        switch (m_units) {
            case MSR::M_UNITS_NONE:
                result = "none";
                break;
            default:
                GEOPM_DEBUG_ASSERT(false, "invalid units enum");
                break;
        }
        // TODO: others
        // {"seconds", M_UNITS_SECONDS},
        // {"hertz", M_UNITS_HERTZ},
        // {"watts", M_UNITS_WATTS},
        // {"joules", M_UNITS_JOULES},
        // {"celsius", M_UNITS_CELSIUS}

        return result;
    }

    std::function<std::string(double)> MSRFieldSignal::format_function(void) const
    {
        return m_format_func;
    }

    void MSRFieldSignal::setup_batch(void)
    {
        //GEOPM_DEBUG_ASSERT(m_msrio != nullptr, "no valid MSRIO object.");

        //m_data = m_msrio->add_read(m_cpu, m_offset);
        m_raw_msr->setup_batch();

        //GEOPM_DEBUG_ASSERT(m_data, "no memory mapped for signal value.");
    }

    double MSRFieldSignal::convert_raw_value(double val)
    {
        uint64_t field = geopm_signal_to_field(val);
        uint64_t subfield = (field & m_mask) >> m_shift;
        uint64_t subfield_last = (m_last_field & m_mask) >> m_shift;
        double result = NAN;

        switch (m_function) {
            case MSR::M_FUNCTION_OVERFLOW:
                if (subfield_last > subfield) {
                    ++m_num_overflow;
                }
                result = subfield + ((m_subfield_max + 1.0) * m_num_overflow);
                break;
            case MSR::M_FUNCTION_SCALE:
                result = subfield;
                break;
            default:
                GEOPM_DEBUG_ASSERT(false, "invalid function type for MSRFieldSignal");
                break;
        }
        result *= m_scalar;
        m_last_field = field;
        return result;

        /*
        uint64_t float_y, float_z;
        switch (m_function) {
            case MSR::M_FUNCTION_LOG_HALF:
                // F = S * 2.0 ^ -X
                result = 1.0 / (1ULL << subfield);
                break;
            case MSR::M_FUNCTION_7_BIT_FLOAT:
                // F = S * 2 ^ Y * (1.0 + Z / 4.0)
                // Y in bits [0:5) and Z in bits [5:7)
                float_y = subfield & 0x1F;
                float_z = subfield >> 5;
                result = (1ULL << float_y) * (1.0 + float_z / 4.0);
                break;
            case MSR::M_FUNCTION_OVERFLOW:
                if (subfield_last > subfield) {
                    ++num_overflow;
                }
                result = subfield + ((m_subfield_max + 1.0) * num_overflow);
                break;
            case MSR::M_FUNCTION_SCALE:
                result = subfield;
                break;
            default:
                break;
        }
        result *= m_scalar;
        last_field = field;
        return result;
        */

        //return ((geopm_signal_to_field(val) & m_mask) >> m_shift) * m_scalar;
    }

    double MSRFieldSignal::sample(void)
    {
        //GEOPM_DEBUG_ASSERT(m_data != nullptr, "no memory mapped for signal value.");
        return convert_raw_value(m_raw_msr->sample());
        // convert to double
        //return geopm_field_to_signal(*m_data);
    }

    double MSRFieldSignal::read(void)
    {
        //GEOPM_DEBUG_ASSERT(m_msrio != nullptr, "no valid MSRIO object.");
        return convert_raw_value(m_raw_msr->read());
    }
}

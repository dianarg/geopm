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

#ifndef MSRFIELDSIGNAL_HPP_INCLUDE
#define MSRFIELDSIGNAL_HPP_INCLUDE

#include <cstdint>
#include <cmath>

#include <string>
#include <memory>

#include "Signal.hpp"
#include "MSRIO.hpp"

namespace geopm
{

    /// @todo: copied from MSR class.  Okay to include MSR.hpp to get these
    // would be nice if these were enum class?  also no max for bounds check
    // enum m_function_e {
    //     M_FUNCTION_SCALE,           // Only apply scalar value (applied by all functions)
    //     M_FUNCTION_LOG_HALF,        // 2.0 ^ -X
    //     M_FUNCTION_7_BIT_FLOAT,     // 2 ^ Y * (1.0 + Z / 4.0) : Y in [0:5), Z in [5:7)
    //     M_FUNCTION_OVERFLOW,        // Counter that may overflow
    // };

    // enum m_units_e {
    //     M_UNITS_NONE,
    //     M_UNITS_SECONDS,
    //     M_UNITS_HERTZ,
    //     M_UNITS_WATTS,
    //     M_UNITS_JOULES,
    //     M_UNITS_CELSIUS,
    // };


    /// Encapsulates conversion of MSR bitfields to double signal
    /// values in SI units.

    /// @todo: most implementation is the same as MSREncode class.
    /// The hope is that this class can eventually replace the use of
    /// MSREncode.
    class MSRFieldSignal : public Signal
    {
        public:
            MSRFieldSignal(std::shared_ptr<Signal> raw_msr,
                           int begin_bit,
                           int end_bit,
                           int function,
                           double scalar,
                           int units,
                           std::function<std::string(double)> format_func);
            int domain(void) const override;
            std::string units(void) const override;
            void setup_batch(void) override;
            double sample(void) override;
            double read(void) override;
            std::function<std::string(double)> format_function(void) const override;
        private:
            double convert_raw_value(double val);
            /// Underlying raw MSR that contains the field.  If it
            /// becomes too expensive to have another layer of
            /// indirection, this can be replaced with a pointer to
            /// the MSRIO and an implementation similar to
            /// RawMSRSignal.  This should be a RawMSRSignal in most
            /// cases but a base class pointer is used for testing and only the
            /// public interface is used.
            std::shared_ptr<Signal> m_raw_msr;
            const int m_shift;
            const int m_num_bit;
            const uint64_t m_mask;
            const uint64_t m_subfield_max;
            const int m_function;
            const double m_scalar;
            uint64_t m_last_field;
            int m_num_overflow;
            int m_units;
            std::function<std::string(double)> m_format_func;
    };
}

#endif

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

#ifndef SIGNAL_HPP_INCLUDE
#define SIGNAL_HPP_INCLUDE

namespace geopm
{
    /// An abstract interface for all types of signals supported by an IOGroup.
    /// Any implementation specific data should be injected in the derived class
    /// constructor and used in setup_batch() if necessary.
    /// @todo const methods could all be implemented as maps in the owning IOGroup.
    ///       these may not be needed for reading the signal.  Units and
    ///       description are already moved out.
    class Signal
    {
        public:
            virtual ~Signal() = default;
            /// @brief The domain of the signal.
            virtual int domain(void) const = 0;
            /// @brief The function used to display this signal.
            virtual std::function<std::string(double)> format_function(void) const = 0;
            /// @brief Prepare the signal for being updating through
            ///        side effects by the owner's read_batch step.
            ///        This method should not fail if called multiple
            ///        times, and ideally only apply the side effects
            ///        on the first call.
            virtual void setup_batch(void) = 0;
            /// @brief Apply any conversions necessary to interpret
            ///        the latest stored value as a double.
            virtual double sample(void) = 0;
            /// @brief Directly the value of the signal without
            ///        affecting any pushed batch signals.
            virtual double read(void) = 0;
    };
}

#endif

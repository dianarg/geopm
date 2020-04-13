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

#ifndef RAWMSRSIGNAL_HPP_INCLUDE
#define RAWMSRSIGNAL_HPP_INCLUDE

#include <cstdint>
#include <cmath>

#include <string>
#include <memory>

#include "Signal.hpp"
#include "MSRIO.hpp"

namespace geopm
{
    class RawMSRSignal : public Signal
    {
        public:
            RawMSRSignal(std::shared_ptr<MSRIO> msrio,
                         int cpu,
                         uint64_t offset);
            RawMSRSignal(const RawMSRSignal &other);
            virtual ~RawMSRSignal() = default;
            std::unique_ptr<Signal> clone(void) const override;
            void setup_batch(void) override;
            double sample(void) override;
            double read(void) override;
        private:
            /// MSRIO object shared by all MSR signals in the same
            /// batch.  This object should outlive all other data in
            /// the Signal.
            std::shared_ptr<MSRIO> m_msrio;
            int m_cpu;
            uint64_t m_offset;
            /// Location of the data that will be updated by the
            /// MSRIO's read_batch() calls.
            uint64_t *m_data;
            bool m_is_batch_ready;
    };
}

#endif

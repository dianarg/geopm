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

#ifndef SIGNAL_HPP_INCLUDE
#define SIGNAL_HPP_INCLUDE

namespace geopm
{
    class ISignal
    {
        public:
            /// @todo Maybe this class should be a two parameter template.
            // The union, enum pair is kind of dancing around the issue.
            union m_signal_u {
                double f;
                uint64_t i;
                void *p;
            };
            enum m_signal_e {
                M_SIGNAL_FLOAT,
                M_SIGNAL_INTEGER,
                M_SIGNAL_POINTER,
            };
            ISignal(void) {}
            virtual ~ISignal(void) {}
            virtual void name(std::string &signal_name) = 0;
            virtual void sample(const std::vector<uint64_t> &encoded, m_signal_u &value) = 0;
            virtual void decode(const std::vector<uint64_t> &encoded, std::vector<union m_signal_u> &decoded) = 0;
            virtual void reduce(const std::vector<union m_signal_u> &decoded, union m_signal_u &value) = 0;
    };

    class Signal : public ISignal
    {
        public:
            Signal(size_t num_encoded, ISignal::m_signale_e encoded_type, size_t num_decoded,  ISignal::m_signal_e decoded_type);
            virtual ~Signal();
            virtual void name(std::string &signal_name) = 0;
            virtual void sample(const std::vector<uint64_t> &encoded, ISignal::m_signal_u &value);
            virtual void decode(const std::vector<uint64_t> &encoded, std::vector<ISignal::m_signal_u> &decoded) = 0;
            virtual void reduce(const std::vector<ISignal::m_signal_u> &decoded, ISignal::m_signal_u &value);
        protected:
            size_t m_num_encoded;
            ISignal::m_signale_e m_encoded_type;
            size_t m_num_decoded;
            ISignal::m_signal_e m_decoded_type);
    };

}

#endif

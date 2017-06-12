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
    template <class etype, class dtype>
    class ISignal
    {
        public:
            ISignal(void) {}
            virtual ~ISignal(void) {}
            virtual void name(std::string &signal_name) = 0;
            virtual void sample(const std::vector<etype> &encoded, dtype &value) = 0;
            virtual void decode(const std::vector<etype> &encoded, std::vector<dtype> &decoded) = 0;
            virtual void reduce(const std::vector<dtype> &decoded, dtype &value) = 0;
            virtual void reduce(const std::vector<dtype> &decoded, dtype &value, geopm_time_s target_time);
    };

    template <class etype, class dtype>
    class Signal : public ISignal
    {
        public:
            Signal(const std::string &signal_name, size_t num_encoded, size_t num_decoded)
                : m_signal_name(signal_name)
                , m_num_encoded(num_encoded)
                , m_num_decoded(num_decoded)
            {

            }
            virtual ~Signal()
            {

            }
            virtual void name(std::string &signal_name)
            {
                signal_name = m_signal_name;
            }
            virtual void sample(const std::vector<etype> &encoded, dtype &value)
            {
                std::vector<dtype>decoded(m_num_decoded);
                decode(encoded, decoded);
                return reduce(decoded);
            }
            virtual void decode(const std::vector<etype> &encoded, std::vector<dtype> &decoded) = 0;
            virtual void reduce(const std::vector<dtype> &decoded, dtype &value)
            {
                value = decoded[0];
                if (m_num_decoded > 1) {
                    value = std::accumulate(decoded.begin() + 1, decoded.end(), value);
                }
            }
            virtual void reduce(const std::vector<dtype> &decoded, dtype &value, geopm_time_s target_time)
            {
                reduce(decoded, value);
            }
        protected:
            std::string m_name;
            size_t m_num_encoded;
            size_t m_num_decoded;
    };

}

#endif

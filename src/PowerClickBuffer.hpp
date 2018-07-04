/*
 * Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
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

namespace geopm
{
    class IPowerClickBuffer
    {
        public:
            IPowerClickBuffer(int num_children, int max_sample) = default;
            virtual ~IPowerClickBuffer() = default;
            virtual void update(double power, double runtime) = 0;
            virtual double runtime_sample_runtime(double power) const = 0;
            virtual int runtime_num_sample(double power) const = 0;
            virtual double runtime_stddev(double power) const = 0;
    };

    class PowerClickBuffer
    {
        public:
            PowerClickBuffer(int num_children, int max_sample);
            virtual ~PowerClickBuffer() = default;
            void update(double power, double runtime) override;
            double runtime_sample_runtime(double power) const override;
            int runtime_num_sample(double power) const override;
            double runtime_stddev(double power) const override;
        private:
            /// @brief Map from power limit to circular buffers
            ///        containing runtime measurements.
            std::map<double, CircularBuffer<double> > m_power_runtime_map;
    };
}

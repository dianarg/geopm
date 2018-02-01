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
#ifndef TIMEIOGROUP_HPP_INCLUDE
#define TIMEIOGROUP_HPP_INCLUDE

#include "IOGroup.hpp"
#include "geopm_time.h"

namespace geopm
{
    class TimeIOGroup : public IOGroup
    {
        public:
            TimeIOGroup();
            virtual ~TimeIOGroup();
            virtual bool is_valid_signal(const std::string &signal_name) override;
            virtual bool is_valid_control(const std::string &control_name) override;
            virtual int push_signal(const std::string &signal_name, int domain_type, int domain_idx)  override;
            virtual int push_control(const std::string &control_name, int domain_type, int domain_idx) override;
            virtual void read_batch(void) override;
            virtual void write_batch(void) override;
            virtual double sample(int batch_idx) override;
            virtual void adjust(int batch_idx, double setting) override;
            virtual double read_signal(const std::string &signal_name, int domain_type, int domain_idx) override;
            virtual void write_control(const std::string &control_name, int domain_type, int domain_idx, double setting) override;
        protected:
            bool m_is_signal_pushed;
            geopm_time_s m_time_zero;
            double m_time_curr;
    };
}

#endif

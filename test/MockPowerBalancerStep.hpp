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

#ifndef MOCKPOWERBALANCERSTEP_HPP_INCLUDE
#define MOCKPOWERBALANCERSTEP_HPP_INCLUDE

#include "PowerBalancerAgent.hpp"

class MockPowerBalancerStep : public geopm::IPowerBalancerStep
{
    public:
        MOCK_METHOD0(reset,
                     int(void));
        MOCK_CONST_METHOD0(step_count,
                           int(void));
        MOCK_METHOD0(inc_step_count,
                     void(void));
        MOCK_CONST_METHOD1(is_send_down_limit,
                     bool(int step));
        MOCK_CONST_METHOD0(is_send_down_limit,
                     bool(void));
        MOCK_CONST_METHOD0(is_measure_runtime,
                     bool(void));
        MOCK_CONST_METHOD0(is_reduce_limit,
                     bool(void));
        MOCK_CONST_METHOD0(is_step_complete,
                     bool(void));
        MOCK_METHOD1(is_step_complete,
                     void(bool is_complete));
        MOCK_METHOD1(step,
                     void(int step));
};

#endif

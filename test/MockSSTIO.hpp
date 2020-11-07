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

#ifndef MOCKSSTIO_HPP_INCLUDE
#define MOCKSSTIO_HPP_INCLUDE

#include "gmock/gmock.h"

#include "SSTIO.hpp"


class MockSSTIO : public geopm::SSTIO
{
    public:
        MOCK_METHOD5(add_mbox_read,
                     int(uint32_t cpu_index, uint32_t command,
                         uint32_t subcommand, uint32_t subcommand_arg,
                         uint32_t interface_parameter));
        MOCK_METHOD5(add_mbox_write,
                     int(uint32_t cpu_index, uint32_t command,
                         uint32_t subcommand,
                         uint32_t interface_parameter, uint32_t write_value));
        // MOCK_METHOD2(mmio_read,
        //              uint32_t(uint32_t cpu_index, uint32_t register_offset));
        // MOCK_METHOD3(mmio_write,
        //              void(uint32_t cpu_index, uint32_t register_offset,
        //                   uint32_t value));

        MOCK_METHOD0(read_batch,
                     void(void));
        MOCK_CONST_METHOD1(sample,
                           uint32_t(int index));
        MOCK_METHOD3(adjust,
                     void(int index, uint32_t write_value, uint64_t mask));
};

#endif

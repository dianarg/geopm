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

#include "geopm_test.hpp"

#include "geopm_hash.h"

bool is_format_double(std::function<std::string(double)> func)
{
    double value = (double)0x3FF00000000000ULL;
    std::string expected = "1.799680632343757e+16";
    return func(value) == expected;
}

bool is_format_float(std::function<std::string(double)> func)
{
    double value = (double)0x3FF00000000000ULL;
    std::string expected = "1.79968e+16";
    return func(value) == expected;
}

bool is_format_integer(std::function<std::string(double)> func)
{
    double value = (double)0x3FF00000000000ULL;
    std::string expected = "17996806323437568";
    return func(value) == expected;
}

bool is_format_hex(std::function<std::string(double)> func)
{
    double value = (double)0x3FF00000000000ULL;
    std::string expected = "0x003ff00000000000";
    return func(value) == expected;
}

bool is_format_raw64(std::function<std::string(double)> func)
{
    uint64_t value_i = 0x3FF00000000000ULL;
    double value = geopm_field_to_signal(value_i);
    std::string expected = "0x003ff00000000000";
    return func(value) == expected;
}

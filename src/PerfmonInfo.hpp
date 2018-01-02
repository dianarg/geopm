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

#ifndef PERFMONINFO_HPP_INCLUDE
#define PERFMONINFO_HPP_INCLUDE

#include <map>
#include <stdexcept>
#include <string>

namespace geopm {

    /// Used to hold information about performance counters, as described in
    /// files found at https://download.01.org/perfmon/
    class PerfmonEvents
    {
        public:
            PerfmonEvents(std::string json_str);
            virtual ~PerfmonEvents();
            uint64_t code(const std::string &event_name);
            uint64_t mask(const std::string &event_name);
            bool is_off_core(const std::string &event_name);
            uint64_t off_core_offset0(const std::string &event_name);
            uint64_t off_core_offset1(const std::string &event_name);
            uint64_t off_core_code0(const std::string &event_name);
            uint64_t off_core_code1(const std::string &event_name);
        protected:
            struct m_event {
                uint64_t code;
                uint64_t mask;
                uint64_t off_core_offset[2];
                uint64_t off_core_code[2];
            }
            std::map<std::string, stuct m_event> m_event_map;
    }
}

#endif

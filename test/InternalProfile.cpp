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

#include <sstream>

#include "InternalProfile.hpp"
#include "Exception.hpp"
#include "config.h"
#include <iostream>
namespace geopm
{
    InternalProfile &InternalProfile::internal_profile(void)
    {
        static InternalProfile instance;
        return instance;
    }

    InternalProfile::InternalProfile()
        : m_region_stack_colon(std::string::npos)
        , m_region_map_curr_it(m_region_map.end())

    {

    }

    void InternalProfile::enter(const std::string &region_name)
    {
        if (m_region_stack.size()) {
            m_region_stack_colon = m_region_stack.size();
            m_region_stack += ":" + region_name;
        }
        else {
            m_region_stack = region_name;
        }
        m_region_map_curr_it = m_region_map.emplace(std::piecewise_construct,
                                                    std::forward_as_tuple(m_region_stack),
                                                    std::forward_as_tuple(M_REGION_ZERO)).first;
        geopm_time(&(m_region_map_curr_it->second.enter_time));
    }

    void InternalProfile::exit(const std::string &region_name)
    {
        struct geopm_time_s curr_time;
        geopm_time(&curr_time);
        if (m_region_map_curr_it == m_region_map.end()) {
            for (auto temp : m_region_map) {
                std::cout << temp.first << std::endl;
            }

            throw Exception("InternalProfile::exit(): Region name has not been previously passed to the enter() method: " + region_name,
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        auto region_time = geopm_time_diff(&(m_region_map_curr_it->second.enter_time), &curr_time);
        m_region_map_curr_it->second.total_time += region_time;
        m_region_map_curr_it->second.min_time = std::min(region_time, m_region_map_curr_it->second.min_time);
        m_region_map_curr_it->second.max_time = std::max(region_time, m_region_map_curr_it->second.max_time);
        ++(m_region_map_curr_it->second.count);
        if (m_region_stack_colon != std::string::npos) {
            m_region_stack = m_region_stack.substr(0, m_region_stack_colon);
            m_region_stack_colon = m_region_stack.find(':');
            m_region_map_curr_it = m_region_map.find(m_region_stack);
        }
        else {
            m_region_stack = "";
            m_region_map_curr_it = m_region_map.end();
        }
    }

    std::string InternalProfile::report(void)
    {
        std::ostringstream result;
        result << "region-name \t| time \t| count \t| mean \t| min \t| max \n";
        for (auto it = m_region_map.begin(); it != m_region_map.end(); ++it) {
            result << it->first << " \t| "
                   << it->second.total_time << " \t| "
                   << it->second.count << " \t| "
                   << it->second.total_time / it->second.count << " \t| "
                   << it->second.min_time << " \t| "
                   << it->second.max_time
                   << "\n";
        }
        result << "\n";
        return result.str();
    }
}

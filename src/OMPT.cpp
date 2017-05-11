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

#include <string>
#include <iostream>
#include <fstream>
#include <map>
#include <sstream>
#include <iomanip>

#include "geopm.h"
#include "geopm_omp.h"
#include "geopm_message.h"
#include "geopm_error.h"
#include "Exception.hpp"
#include "config.h"

#ifdef _OPENMP
#include <omp.h>

#ifdef GEOPM_HAS_OMPT
#include <ompt.h>


namespace geopm
{
    class OMPT
    {
        public:
            OMPT();
            virtual ~OMPT();
            uint64_t region_id(void *parallel_function);
            void region_name(void *parallel_function, std::string &name);
        private:
            /// Map from <virtual_address, is_end> pair representing
            /// half of a virtual address range to the object file
            /// asigned to the address range.
            std::map<std::pair<size_t, bool>, std::string> m_range_object_map;
            /// Map from function address to geopm region ID
            std::map<size_t, uint64_t> m_function_region_id_map;
            const std::string &object_name(void *function_ptr);
            size_t virtual_offset(void *function_ptr);
    };

    static OMPT &ompt(void)
    {
        static OMPT instance;
        return instance;
    }

    class MapParseException : public Exception
    {
        public:
            MapParseException() {}
            virtual ~MapParseException() {}
    };

    OMPT::OMPT()
    {
        std::ifstream maps_stream("/proc/self/maps");
        while (maps_stream.good()) {
            try {
                int err = 0;
                std::string line;
                std::getline(maps_stream, line);
                if (line.length() == 0) {
                    throw MapParseException();
                }
                size_t addr_begin, addr_end;
                int n_scan = sscanf(line.c_str(), "%zx-%zx", &addr_begin, &addr_end);
                if (n_scan != 2) {
                    throw MapParseException();
                }

                std::string object;
                size_t object_loc = line.rfind(' ') + 1;
                if (object_loc == std::string::npos) {
                    throw MapParseException();
                }
                object = line.substr(object_loc);
                if (line.find(" r-xp ") != line.find(' ')) {
                    throw MapParseException();
                }
                std::pair<size_t, bool> aa(addr_begin, false);
                std::pair<size_t, bool> bb(addr_end, true);
                std::pair<std::pair<size_t, bool>, std::string> cc(aa, object);
                std::pair<std::pair<size_t, bool>, std::string> dd(bb, object);
                auto it0 = m_range_object_map.insert(m_range_object_map.begin(), cc);
                auto it1 = m_range_object_map.insert(it0, dd);
                ++it0;
                if (it0 != it1) {
                    throw Exception("Error parsing /proc/self/maps, overlapping address ranges.",
                                    GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
                }
            }
            catch (MapParseException) {

            }
        }
    }

    OMPT::~OMPT()
    {

    }

    uint64_t OMPT::region_id(void *parallel_function)
    {
        uint64_t result = GEOPM_REGION_ID_UNDEFINED;
        auto it = m_function_region_id_map.find((size_t)parallel_function);
        if (m_function_region_id_map.end() != it) {
            result = (*it).second;
        }
        else {
            std::string rn;
            region_name(parallel_function, rn);
            int err = geopm_prof_region(rn.c_str(), GEOPM_REGION_HINT_UNKNOWN, &result);
            if (err) {
                result = GEOPM_REGION_ID_UNDEFINED;
            }
            else {
                m_function_region_id_map.insert(std::pair<size_t, uint64_t>((size_t)parallel_function, result));
            }
        }
        return result;
    }

    void OMPT::region_name(void *parallel_function, std::string &name)
    {
        name.clear();
        auto it = m_range_object_map.lower_bound(std::pair<size_t, bool>((size_t)parallel_function, false));
        auto it_next = it;
        it_next++;
        if (std::distance(it, m_range_object_map.end()) > 1 &&
            false == (*it).first.second &&
            true == (*it_next).first.second) {
            size_t offset = (size_t)parallel_function - (size_t)((*it).first.first);
            std::ostringstream name_stream("OMPT-");
            name_stream << (*it).second << "-0x" << std::uppercase << std::setfill('0') << std::setw(16) << std::hex << offset;
            name = name_stream.str();
        }
    }
}


extern "C"
{
    static void *g_curr_parallel_function = NULL;
    static ompt_parallel_id_t g_curr_parallel_id;
    static uint64_t g_curr_region_id = GEOPM_REGION_ID_UNDEFINED;

    static void on_ompt_event_parallel_begin(ompt_task_id_t parent_task_id, ompt_frame_t *parent_task_frame,
                                             ompt_parallel_id_t parallel_id, uint32_t requested_team_size,
                                             void *parallel_function, ompt_invoker_t invoker)
     {
          if (g_curr_parallel_function != parallel_function) {
              g_curr_parallel_function = parallel_function;
              g_curr_parallel_id = parallel_id;
              g_curr_region_id = geopm::ompt().region_id(parallel_function);
          }
          if (g_curr_region_id != GEOPM_REGION_ID_UNDEFINED) {
              geopm_prof_enter(g_curr_region_id);
          }
     }

     static void on_ompt_event_parallel_end(ompt_parallel_id_t parallel_id, ompt_task_id_t task_id,
                                            ompt_invoker_t invoker)
     {
          if (g_curr_region_id != GEOPM_REGION_ID_UNDEFINED &&
              g_curr_parallel_id == parallel_id) {
              geopm_prof_exit(g_curr_region_id);
          }
     }


     void ompt_initialize(ompt_function_lookup_t lookup, const char *runtime_version, unsigned int ompt_version)
     {
         ompt_set_callback_t ompt_set_callback = (ompt_set_callback_t) lookup("ompt_set_callback");
         ompt_set_callback(ompt_event_parallel_begin, (ompt_callback_t) &on_ompt_event_parallel_begin);
         ompt_set_callback(ompt_event_parallel_end, (ompt_callback_t) &on_ompt_event_parallel_end);

     }

     ompt_initialize_t ompt_tool()
     {
         return &ompt_initialize;
     }
}


#endif
#endif

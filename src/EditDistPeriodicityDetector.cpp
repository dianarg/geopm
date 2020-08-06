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

#include "config.h"
#include "EditDistPeriodicityDetector.hpp"

#include <vector>
#include <cmath>
#include <algorithm>

#include "record.hpp"
#include "geopm_debug.hpp"

#include "InternalProfile.hpp"

namespace geopm
{
    EditDistPeriodicityDetector::EditDistPeriodicityDetector(int history_buffer_size)
        : m_history_buffer(history_buffer_size)
        , history_buffer_size(history_buffer_size)
        , m_period(-1)
        , m_score(-1)
        , nn(0)
    {
        DP = new unsigned int[history_buffer_size * history_buffer_size * history_buffer_size];
        myinf = 2*history_buffer_size;
    }

    void
    EditDistPeriodicityDetector::Dset(int i, int j, int m, unsigned int val) {
        DP[((i % history_buffer_size) * history_buffer_size + (j % history_buffer_size)) * history_buffer_size + (m % history_buffer_size)] = val;
    }

    unsigned int
    EditDistPeriodicityDetector::Dget(int i, int j, int m) {
        if (i <= nn - history_buffer_size) {
            return myinf;
        }
        if (j >= history_buffer_size) {
            return myinf;
        }
        if (m <= nn - history_buffer_size) {
            return myinf;
        }
        return DP[((i % history_buffer_size) * history_buffer_size
                + (j % history_buffer_size)) * history_buffer_size + (m % history_buffer_size)];
    }


    void EditDistPeriodicityDetector::update(const record_s &record)
    {
        if (record.event == EVENT_REGION_ENTRY) {
            m_history_buffer.insert(record.signal);
            nn ++;
            calc_period();
        }
    }

    void EditDistPeriodicityDetector::calc_period(void)
    {
        int NN = m_history_buffer.size();

ip_enter(__func__);
        for (int ii = std::max({0, nn-history_buffer_size}); ii < nn; ++ii) {
            Dset(ii, 0, nn-1, 0);
        }
        for (int mm = std::max({0, nn-history_buffer_size}); mm < nn; ++mm) {
            Dset(0, nn-mm, mm, nn-mm);
        }


        for (int mm = std::max({1, nn-history_buffer_size}); mm < nn; ++mm) {
            for (int ii = std::max({1, nn-history_buffer_size}); ii <= mm; ++ii) {
                int term = 2;
                if (nn-(ii-1) <= NN) {
                    term = (m_history_buffer.value(NN-(nn-(ii-1))) !=
                            m_history_buffer.value(NN - 1)) ?
                           2 : 0;
                }
                Dset(ii, nn-mm, mm,
                        std::min({Dget(ii - 1, nn - mm    , mm) + 1,
                                  Dget(ii    , nn - mm - 1, mm) + 1,
                                  Dget(ii - 1, nn - mm - 1, mm) + term}));
            }
        }

        if (NN < 2) {
            return;
        }

        int mm = std::max({(int)(nn / 2.0 + 0.5), nn-history_buffer_size});
        int bestm = mm;
        unsigned int bestval = Dget(mm, nn - mm, mm);
        ++mm;
        for(; mm < nn; ++mm) {
            unsigned int val = Dget(mm, nn - mm, mm);
            if(val < bestval) {
                bestval = val;
                bestm = mm;
            }
        }
        m_score = bestval;
        // Originally this was:
        //      m_period = n - bestm + 1;
        // However since the algorithm find the bestm with the lowest index it will
        // return a string with a repeating pattern in it. For example:
        //     A B A B A B ...
        // find_min_match will find the smallest repeating pattern in it: A B
ip_exit(__func__);
        m_period = find_min_match(NN-(nn-bestm));
    }

    int EditDistPeriodicityDetector::get_period(void) const
    {
        return m_period;
    }

    int EditDistPeriodicityDetector::get_score(void) const
    {
        return m_score;
    }

    uint64_t EditDistPeriodicityDetector::get_history_value(int index) const
    {
        return m_history_buffer.value(index - 1);
    }

    int EditDistPeriodicityDetector::find_min_match(int slice_start) const
    {
        if (m_history_buffer.size() == slice_start) {
            return 1;
        }
ip_enter(__func__);
        std::vector<uint64_t> recs = m_history_buffer.make_vector(
            slice_start, m_history_buffer.size());
        int result = recs.size();
        bool perfect_match = false;
        int div_max = (recs.size() / 2) + 1;
        for (int div = 1; !perfect_match && div < div_max; ++div) {
            if (recs.size() % div == 0) {
                perfect_match = true;
                int group_max = recs.size() / div;
                for (int group = 1; perfect_match && group < group_max; ++group) {
                    auto cmp1_begin = recs.begin() + div * (group - 1);
                    auto cmp1_end = recs.begin() + div * group;
                    auto cmp2_begin = cmp1_end;
                    if (!std::equal(cmp1_begin, cmp1_end, cmp2_begin)) {
                        perfect_match = false;
                    }
                }
                if (perfect_match) {
                    result = div;
                }
            }
        }
ip_exit(__func__);
        return result;
    }
}

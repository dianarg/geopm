/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
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

#ifndef MOCKTREECOMM_HPP_INCLUDE
#define MOCKTREECOMM_HPP_INCLUDE

#include "gmock/gmock.h"

#include <set>

#include "TreeComm.hpp"

class MockTreeComm : public geopm::TreeComm
{
    public:
        MockTreeComm();
        virtual ~MockTreeComm();

        MOCK_CONST_METHOD0(num_level_controlled,
                           int(void));
        MOCK_CONST_METHOD0(max_level,
                           int(void));
        MOCK_CONST_METHOD0(root_level,
                           int(void));
        MOCK_CONST_METHOD1(level_rank,
                           int(int level));
        MOCK_CONST_METHOD1(level_size,
                           int(int level));

        void send_up(int level, const std::vector<double> &sample) override;
        void send_up_mock_child(int level, int child_idx,
                                const std::vector<double> &sample);
        void send_down(int level, const std::vector<std::vector<double> > &policy) override;
        bool receive_up(int level, std::vector<std::vector<double> > &sample) override;
        bool receive_down(int level, std::vector<double> &policy) override;
        MOCK_CONST_METHOD0(overhead_send,
                     size_t(void));
        MOCK_METHOD1(broadcast_string,
                     void(const std::string &str));
        MOCK_METHOD0(broadcast_string,
                     std::string(void));
        int num_send(void);
        int num_recv(void);
        std::set<int> levels_sent_down(void);
        std::set<int> levels_rcvd_down(void);
        std::set<int> levels_sent_up(void);
        std::set<int> levels_rcvd_up(void);
        void reset_spy(void);
    private:
        // map from level -> last sent data
        std::map<int, std::vector<double> > m_data_sent_up;
        std::map<int, std::vector<double> > m_data_sent_down;
        // map from level, child -> last sent data
        std::map<std::pair<int, int>, std::vector<double> > m_data_sent_up_child;
        int m_num_send = 0;
        int m_num_recv = 0;
        std::set<int> m_levels_sent_down;
        std::set<int> m_levels_rcvd_down;
        std::set<int> m_levels_sent_up;
        std::set<int> m_levels_rcvd_up;
};

#endif

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
#include "MockTreeComm.hpp"

// Nothing special is explicitly happening here, but this ensures that the
// constructors are not inlined every place they are used. It is expected
// to reduce average build time of unit tests.
MockTreeComm::MockTreeComm() {}
MockTreeComm::~MockTreeComm() {}

void MockTreeComm::send_up(int level, const std::vector<double> &sample)
{
    ++m_num_send;
    m_levels_sent_up.insert(level);
    m_data_sent_up[level] = sample;
}

void MockTreeComm::send_up_mock_child(int level, int child_idx,
                                      const std::vector<double> &sample)
{
    m_data_sent_up_child[{ level, child_idx }] = sample;
}

void MockTreeComm::send_down(int level, const std::vector<std::vector<double> > &policy)
{
    ++m_num_send;
    m_levels_sent_down.insert(level);
    if (policy.size() == 0) {
        throw std::runtime_error("MockTreeComm::send_down(): policy vector was wrong size");
    }
    m_data_sent_down[level] = policy[0]; /// @todo slightly wrong
}

bool MockTreeComm::receive_up(int level, std::vector<std::vector<double> > &sample)
{
    if (m_data_sent_up.find(level) == m_data_sent_up.end()) {
        return false;
    }
    ++m_num_recv;
    m_levels_rcvd_up.insert(level);
    int child_idx = 0;
    for (auto &vec : sample) {
        if (m_data_sent_up_child.find({ level, child_idx }) !=
            m_data_sent_up_child.end()) {
            vec = m_data_sent_up_child.at({ level, child_idx });
        }
        else {
            vec = m_data_sent_up.at(level);
        }
        ++child_idx;
    }
    return true;
}

bool MockTreeComm::receive_down(int level, std::vector<double> &policy)
{
    if (m_data_sent_down.find(level) == m_data_sent_down.end()) {
        return false;
    }
    ++m_num_recv;
    m_levels_rcvd_down.insert(level);
    policy = m_data_sent_down.at(level);
    return true;
}

int MockTreeComm::num_send(void)
{
    return m_num_send;
}

int MockTreeComm::num_recv(void)
{
    return m_num_recv;
}

std::set<int> MockTreeComm::levels_sent_down(void)
{
    return m_levels_sent_down;
}

std::set<int> MockTreeComm::levels_rcvd_down(void)
{
    return m_levels_rcvd_down;
}

std::set<int> MockTreeComm::levels_sent_up(void)
{
    return m_levels_sent_up;
}

std::set<int> MockTreeComm::levels_rcvd_up(void)
{
    return m_levels_rcvd_up;
}

void MockTreeComm::reset_spy(void)
{
    m_num_send = 0;
    m_num_recv = 0;
    m_levels_sent_down.clear();
    m_levels_sent_up.clear();
    m_levels_rcvd_down.clear();
    m_levels_rcvd_up.clear();
}

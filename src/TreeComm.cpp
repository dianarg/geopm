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

#include <algorithm>

#include "TreeComm.hpp"
#include "config.h"

namespace geopm
{

     TreeComm::TreeComm(std::shared_ptr<IComm> comm,
                        const std::vector<int> &fan_out,
                        int num_send_up,
                        int num_send_down)
         : TreeComm(comm, fan_out, num_send_up, num_send_down, {})
     {

     }

     TreeComm::TreeComm(std::shared_ptr<IComm> comm,
                        const std::vector<int> &fan_out,
                        int num_send_up,
                        int num_send_down,
                        std::vector<std::unique_ptr<ITreeCommLevel> > mock_level)
        : m_root_level(fan_out.size() + 1)
        , m_num_node(comm->num_rank()) // Assume that comm has one rank per node
        , m_fan_out(fan_out)
        , m_num_send_up(num_send_up)
        , m_num_send_down(num_send_down)
        , m_num_level(num_level(coords))
        , m_level(std::move(mock_level))
                  
    {
        if (mock_level.size() == 0) {
            std::shared_ptr<IComm> comm_cart(comm->split_cart(m_fan_out));
            init_level(comm_cart, m_num_level, m_root_level))
        }
        std::reverse(m_fan_out.begin(), m_fan_out.end());
        comm->barrier();
    }

    TreeComm::num_level(std::vector<int> coords)
    {
        result = 1;
        for (auto it = coords.rbegin(); it != coords.rend() && *it == 0; ++it) {
             ++result;
        }
        return result;
    }

    std::vector<std::unique_ptr<ITreeCommLevel> > TreeComm::init_level(std::shared_ptr<IComm> comm_cart, int num_level, int root_level)
    {
        std::vector<std::unique_ptr<ITreeCommLevel> > result;
        int rank_cart = comm_cart->rank();
        /// @todo change coordinate() to return vector
        std::vector<int> coords(comm_cart->coordinate(rank_cart));
        int level = 0;
        for (; level < num_level && level < root_level; ++level) {
            parent_coords[root_level - 1 - level] = 0;
            m_level.emplace_back(
                new TreeCommLevel(comm_cart->split(
                    comm_cart->cart_rank(parent_coords), rank_cart)));
        }
        for (; level < root_level; ++level) {
            comm_cart->split(IComm::M_SPLIT_COLOR_UNDEFINED, 0);
        }
        return result;
    }

    virtual TreeComm::~TreeComm()
    {

    }

    int TreeComm::num_level(void) const
    {
        return m_num_level;
    }

    int TreeComm::root_level(void) const
    {
        return m_root_level;
    }

    int TreeComm::level_rank(int level) const
    {
        if (level < 0 || level >= m_num_level) {
            throw Exception("TreeCommunicator::level_rank()",
                            GEOPM_ERROR_LEVEL_RANGE, __FILE__, __LINE__);
        }
        return m_level[level]->level_rank();
    }

    int TreeComm::level_size(int level) const
    {
        if (level < 0 || level >= m_root_level) {
            throw Exception("TreeCommunicator::level_size()",
                            GEOPM_ERROR_LEVEL_RANGE, __FILE__, __LINE__);
        }
        // Size at root is one, otherwise use m_fan_out to determine
        // level size above.
        int result = 1;
        if (level != m_root_level - 1) {
            result = m_fan_out[level];
        }
        return result;
    }

    void TreeComm::send_up(int level, const std::vector<double> &sample)
    {
        if (level < 0 || level >= m_num_level) {
            throw Exception("TreeCommunicator::send_up()",
                            GEOPM_ERROR_LEVEL_RANGE, __FILE__, __LINE__);
        }
        m_level[level]->send_up(sample);
    }

    void TreeComm::send_down(int level, const std::vector<double> &policy)
    {
        if (level < 0 || level >= m_num_level) {
            throw Exception("TreeCommunicator::send_down()",
                            GEOPM_ERROR_LEVEL_RANGE, __FILE__, __LINE__);
        }
        m_level[level]->send_down(policy);
    }

    void TreeComm::receive_up(int level, std::vector<double> &sample)
    {
        if (level < 1 || level > m_num_level) {
            throw Exception("TreeCommunicator::receive_up()",
                            GEOPM_ERROR_LEVEL_RANGE, __FILE__, __LINE__);
        }
        m_level[level - 1]->receive_up(sample);
    }

    void TreeComm::receive_down(int level, std::vector<double> &policy)
    {
        if (level < 0 || level >= m_num_level) {
            throw Exception("TreeCommunicator::receive_down()",
                            GEOPM_ERROR_LEVEL_RANGE, __FILE__, __LINE__);
        }
        m_level[level]->receive_down(policy);
    }

    size_t TreeComm::overhead_send(void)
    {
        size_t result = 0;
        for (auto it = m_level.begin(); it != m_level.end(); ++it) {
            result += (*it)->overhead_send();
        }
        return result;
    }
}

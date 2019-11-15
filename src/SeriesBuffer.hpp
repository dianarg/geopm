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

#ifndef SHAREDMEMORYCIRCULARBUFFER_HPP_INCLUDE
#define SHAREDMEMORYCIRCULARBUFFER_HPP_INCLUDE

#include "SharedMemoryImp.hpp"

namespace geopm
{
    template <class type>
    class SharedMemoryCircularBuffer
    {
        public:
            SharedMemoryCircularBuffer(std::string shm_key, size_t capacity);
            virtual ~SharedMemoryCircularBuffer();
            size_t insert(const type value);
            struct {
                /// @brief Total number of elements that can be stored
                size_t capacity;
                /// @brief Total number of elemetns currently stored
                size_t size;
                /// @breif Offset into buffer to the oldest stored element
                size_t begin;
                /// @brief Number of elements inserted that have been overwritten
                size_t num_lost;
            } m_header_s;
        private:
            std::unique_ptr <SharedMemoryImp> m_shmem;
            struct m_header_s *m_header;
            type *m_buffer;
    };

    template <class type>
    class SharedMemoryCircularBufferUser
    {
        public:
            SharedMemoryCircularBufferUser(std::string shm_key, int timeout);
            void sample(size_t last_update, size_t &this_update, std::vector<type> &result);
        private:
            std::unique_ptr <SharedMemoryImp> m_shmem;
            struct SharedMemoryCircularBuffer::m_header_s *m_header;
            type *m_buffer;
    };
}

#endif

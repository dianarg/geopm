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

#include "SharedMemoryCircularBuffer.hpp"

using geopm::SharedMemoryCircularBuffer;
using geopm::SharedMemoryCircularBufferUser;

class SharedMemoryTest : public :: testing :: Test
{
    protected:
        void SetUp();
        void TearDown();
        void config_shmem();
        void config_shmem_u();
        size_t m_size;
        std::shared_ptr<SharedMemoryCircularBuffer<int> > m_shmem;
        std::shared_ptr<SharedMemoryCircularBufferUser<int> > m_shmem_u;
};

void SharedMemoryTest::SetUp()
{
    m_size = 5;
    m_shm_key = "/geopm-shm-foo-SharedMemoryCircularBufferTest-" + std::to_string(getpid());    
}

void SharedMemoryTest::TearDown()
{
    
}

void SharedMemoryTest::config_shmem()
{
    m_shmem = std::make_shared<SharedMemoryCircularBuffer<int>>(m_shm_key, m_size);
}

void SharedMemoryTest::config_shmem_u()
{
    m_shmem_u = std::make_shared<SharedMemoryCircularBufferUser>(m_shm_key, 1); // 1 second timeout
}

TEST_F(SharedMemoryTest, share_data)
{
    std::vector<int> expect;
    std::vector<int> result;
    m_shm_key += "-share_data";
    config_shmem();
    config_shmem_u();
    m_shmem.insert(1);
    m_shmem.insert(0);
    m_shmem.insert(2);
    m_shmem.insert(2);
    m_shmem.insert(-1);
    expect = {1, 0, 2, 2, -1};
    result = {};
    size_t last_update = 0;
    m_shmem_u.sample(last_update, last_update, result);
    EXPECT_EQ(expect, result);
    EXPECT_EQ(4, last_update);
    m_shmem_u.sample(last_update, last_update, result);
    EXPECT_EQ(0, result.size());
    EXPECT_EQ(4, last_update);
    m_shmem_u.sample(0, last_update, result);
    EXPECT_EQ(expect, result);
    EXPECT_EQ(4, last_update);
    m_shmem.update(-2);
    expect = {-2};
    result = {};
    m_shmem_u.sample(last_update, last_update, result);
    EXPECT_EQ(expect, result);
    EXPECT_EQ(5, last_update);
    expect = {0, 2, 2, -1, -2};
    result = {};
    m_shmem_u.sample(0, last_update, result);
    EXPECT_EQ(expect, result);
    EXPECT_EQ(5, last_update);
}

TEST_F(SharedMemoryTest, share_data_ipc)
{
    m_shm_key += "-share_data_ipc";
    size_t shared_data = 0xDEADBEEFCAFED00D;

    pid_t pid = fork();
    if (pid) {
        // parent process
        config_shmem_u();
        sleep(1);
        EXPECT_EQ(memcmp(m_shmem_u->pointer(), &shared_data, m_size), 0);
    } else {
        // child process
        config_shmem();
        memcpy(m_shmem->pointer(), &shared_data, m_size);
        sleep(2);
        exit(0);
    }
}

TEST_F(SharedMemoryTest, lock_shmem)
{
    config_shmem();
    config_shmem_u();

    // mutex is hidden at address before the user memory region
    // normally, this mutex should not be accessed directly.  This test
    // checks that get_scoped_lock() has the expected side effects on the
    // mutex.
    pthread_mutex_t *mutex = (pthread_mutex_t*)((char*)m_shmem->pointer() - sizeof(pthread_mutex_t));

    // mutex starts out lockable
    EXPECT_EQ(0, pthread_mutex_trylock(mutex));
    EXPECT_EQ(0, pthread_mutex_unlock(mutex));

    auto lock = m_shmem->get_scoped_lock();
    // should not be able to lock
    EXPECT_NE(0, pthread_mutex_trylock(mutex));
    GEOPM_EXPECT_THROW_MESSAGE(m_shmem->get_scoped_lock(),
                               EDEADLK, "Resource deadlock avoided");

    // destroy the lock
    lock.reset();

    // mutex should be lockable again
    EXPECT_EQ(0, pthread_mutex_trylock(mutex));
    EXPECT_EQ(0, pthread_mutex_unlock(mutex));
}

TEST_F(SharedMemoryTest, lock_shmem_u)
{
    config_shmem();
    config_shmem_u();

    // mutex is hidden at address before the user memory region
    // normally, this mutex should not be accessed directly.  This test
    // checks that get_scoped_lock() has the expected side effects on the
    // mutex.
    pthread_mutex_t *mutex = (pthread_mutex_t*)((char*)m_shmem_u->pointer() - sizeof(pthread_mutex_t));

    // mutex starts out lockable
    EXPECT_EQ(0, pthread_mutex_trylock(mutex));
    EXPECT_EQ(0, pthread_mutex_unlock(mutex));

    auto lock = m_shmem_u->get_scoped_lock();
    // should not be able to lock
    EXPECT_NE(0, pthread_mutex_trylock(mutex));
    GEOPM_EXPECT_THROW_MESSAGE(m_shmem_u->get_scoped_lock(),
                               EDEADLK, "Resource deadlock avoided");

    // destroy the lock
    lock.reset();

    // mutex should be lockable again
    EXPECT_EQ(0, pthread_mutex_trylock(mutex));
    EXPECT_EQ(0, pthread_mutex_unlock(mutex));
}

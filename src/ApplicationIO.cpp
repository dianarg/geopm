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

#include "ApplicationIO.hpp"
#include "PlatformIO.hpp"
#include "ProfileSampler.hpp"
#include "SampleRegulator.hpp"
#include "RuntimeRegulator.hpp"
#include "ProfileIOSample.hpp"
#include "ProfileIOGroup.hpp"
#include "Helper.hpp"
#include "config.h"

#ifdef GEOPM_HAS_XMMINTRIN
#include <xmmintrin.h>
#endif

namespace geopm
{
    ApplicationIO::ApplicationIO(const std::string &shm_key)
        : m_sampler(new ProfileSampler(M_SHMEM_REGION_SIZE))
        , m_sample_regulator(nullptr)
        , m_do_shutdown(false)
        , m_is_connected(false)
        , m_rank_per_node(-1)  /// @todo
    {

    }

    ApplicationIO::~ApplicationIO()
    {

    }

    void ApplicationIO::connect(void)
    {
        if (!m_is_connected) {
            m_sampler->initialize();
            m_rank_per_node = m_sampler->rank_per_node();
            m_prof_sample.resize(m_sampler->capacity());
            std::vector<int> cpu_rank = m_sampler->cpu_rank();
            m_profile_io_sample = std::make_shared<ProfileIOSample>(cpu_rank);
            platform_io().register_iogroup(geopm::make_unique<ProfileIOGroup>(m_profile_io_sample));
            m_is_connected = true;
        }
    }

    bool ApplicationIO::do_shutdown(void) const
    {
        return m_sampler->do_shutdown();
    }

    std::string ApplicationIO::report_name(void) const
    {
        /// @todo implement me
        return "";
    }

    std::string ApplicationIO::profile_name(void) const
    {
        /// @todo implement me
        return "";
    }

    std::set<std::string> ApplicationIO::region_name_set(void) const
    {
        /// @todo implement me
        return {};
    }

    double ApplicationIO::total_region_runtime(uint64_t region_id) const
    {
        return NAN;
    }

    double ApplicationIO::total_region_mpi_runtime(uint64_t region_id) const
    {
        return NAN;
    }

    double ApplicationIO::total_epoch_runtime(void) const
    {
        return NAN;
    }

    double ApplicationIO::total_app_runtime(void) const
    {
        return NAN;
    }

    double ApplicationIO::total_app_mpi_runtime(void) const
    {
        return NAN;
    }

    int ApplicationIO::total_count(uint64_t region_id) const
    {
        return -1;
    }

    std::shared_ptr<IOGroup> ApplicationIO::profile_io_group(void)
    {
        return nullptr;
    }

    void ApplicationIO::update(std::shared_ptr<IComm> comm)
    {
        size_t length = 0; /// @todo fix
        m_sampler->sample(m_prof_sample, length, comm);
        m_profile_io_sample->update(m_prof_sample.cbegin(), m_prof_sample.cbegin() + length);
    }
}

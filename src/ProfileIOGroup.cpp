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


#include "ProfileIOGroup.hpp"
#include "PlatformTopo.hpp"
#include "Exception.hpp"
#include "config.h"

#define GEOPM_PROFILE_IO_GROUP_PLUGIN_NAME "profile"

namespace geopm
{
    ProfileIOGroup::ProfileIOGroup()
    {

    }

    ProfileIOGroup::~ProfileIOGroup()
    {

    }

    bool ProfileIOGroup::is_valid_signal(const std::string &signal_name)
    {

    }

    bool ProfileIOGroup::is_valid_control(const std::string &control_name)
    {
        return false;
    }

    int ProfileIOGroup::signal_domain_type(const std::string &signal_name)
    {

    }

    int ProfileIOGroup::control_domain_type(const std::string &control_name)
    {
        return PlatformTopo::M_DOMAIN_INVALID;
    }

    int ProfileIOGroup::push_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("ProfileIOGroup::push_signal(): signal_name " + signal_name +
                            "not valid for ProfileIOGroup",
                             GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (m_is_batch_read) {
            throw Exception("ProfileIOGroup::push_signal(): cannot push signal after call to read_batch().",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        m_is_signal_pushed = true;
        return 0;
    }

    int ProfileIOGroup::push_control(const std::string &control_name, int domain_type, int domain_idx)
    {
        throw Exception("ProfileIOGroup::push_control() there are no controls supported by the ProfileIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    void ProfileIOGroup::read_batch(void)
    {

    }

    void ProfileIOGroup::write_batch(void)
    {

    }

    double ProfileIOGroup::sample(int batch_idx)
    {

    }

    void ProfileIOGroup::adjust(int batch_idx, double setting)
    {
        throw Exception("ProfileIOGroup::adjust() there are no controls supported by the ProfileIOGroup",
                         GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    double ProfileIOGroup::read_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        if (!is_valid_signal(signal_name)) {
            throw Exception("ProfileIOGroup:read_signal " + signal_name +
                            "not valid for ProfileIOGroup",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    void ProfileIOGroup::write_control(const std::string &control_name, int domain_type, int domain_idx, double setting)
    {
        throw Exception("ProfileIOGroup::write_control() there are no controls supported by the ProfileIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    std::string ProfileIOGroup::plugin_name(void)
    {
        return GEOPM_PROFILE_IO_GROUP_PLUGIN_NAME;
    }

    std::unique_ptr<IOGroup> ProfileIOGroup::make_plugin(void)
    {
        return std::unique_ptr<IOGroup>(new ProfileIOGroup);
    }
}

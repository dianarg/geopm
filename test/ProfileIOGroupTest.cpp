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

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "ProfileIOGroup.hpp"
#include "PlatformTopo.hpp"
#include "MockProfileIOSample.hpp"

using geopm::IProfileIOSample;
using geopm::ProfileIOGroup;
using geopm::PlatformTopo;

class ProfileIOGroupTest : public :: testing:: Test
{
    public:
        ProfileIOGroupTest();
        virtual ~ProfileIOGroupTest();
    protected:
        std::shared_ptr<geopm::IProfileIOSample> m_mock_pios;
        ProfileIOGroup m_piog;
};


ProfileIOGroupTest::ProfileIOGroupTest()
    : m_mock_pios(std::shared_ptr<IProfileIOSample>(new MockProfileIOSample))
    , m_piog(m_mock_pios)
{

}

ProfileIOGroupTest::~ProfileIOGroupTest()
{

}

TEST_F(ProfileIOGroupTest, is_valid)
{
    EXPECT_TRUE(m_piog.is_valid_signal("PROFILE::REGION_ID"));
    EXPECT_TRUE(m_piog.is_valid_signal("PROFILE::PROGRESS"));
    EXPECT_FALSE(m_piog.is_valid_signal("PROFILE::INVALID_SIGNAL"));
    EXPECT_FALSE(m_piog.is_valid_control("PROFILE::INVALID_CONTROL"));
}

TEST_F(ProfileIOGroupTest, domain_type)
{
    EXPECT_EQ(geopm::PlatformTopo::M_DOMAIN_CPU, m_piog.signal_domain_type("PROFILE::REGION_ID"));
    EXPECT_EQ(geopm::PlatformTopo::M_DOMAIN_CPU, m_piog.signal_domain_type("PROFILE::PROGRESS"));
    EXPECT_EQ(geopm::PlatformTopo::M_DOMAIN_INVALID, m_piog.signal_domain_type("PROFILE::INVALID_SIGNAL"));
    EXPECT_EQ(geopm::PlatformTopo::M_DOMAIN_INVALID, m_piog.control_domain_type("PROFILE::INVALID_CONTROL"));
}

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

#include "Exception.hpp"
#include "RAPLPlatform.hpp"
#include "geopm_message.h"
#include "geopm_time.h"
#include "config.h"

namespace geopm
{
    RAPLPlatform::RAPLPlatform()
        : Platform()
        , m_num_freq_domain(0)
        , m_num_energy_domain(0)
        , m_description("rapl")
        , M_HSX_ID(0x63F)
        , M_IVT_ID(0x63E)
        , M_SNB_ID(0x62D)
        , M_BDX_ID(0x64F)
        , M_KNL_ID(0x657)
    {

    }

    RAPLPlatform::~RAPLPlatform()
    {

    }

    bool RAPLPlatform::is_model_supported(int platform_id, const std::string &description) const
    {
        return ((platform_id == M_IVT_ID ||
                 platform_id == M_SNB_ID ||
                 platform_id == M_BDX_ID ||
                 platform_id == M_KNL_ID ||
                 platform_id == M_HSX_ID) &&
                description == m_description);
    }

    void RAPLPlatform::initialize(void)
    {
        m_num_cpu = m_imp->num_hw_cpu();
        m_num_package = m_imp->num_package();
        m_num_tile = m_imp->num_tile();
        m_num_freq_domain = m_imp->num_control_domain(GEOPM_CONTROL_TYPE_FREQUENCY);
        m_num_power_domain = m_imp->num_control_domain(GEOPM_CONTROL_TYPE_POWER);
        m_batch_desc.resize(m_num_power_domain * m_imp->num_energy_signal() + m_num_freq_domain * m_imp->num_counter_signal());

        int count = 0;
        int freq_domain_per_power_domain = m_num_freq_domain / m_num_power_domain;
        int power_domain = m_imp->power_control_domain();
        int freq_domain = m_imp->performance_counter_domain();
        for (int i = 0; i < m_num_power_domain; i++) {
            m_batch_desc[count].device_type = power_domain;
            m_batch_desc[count].device_index = i;
            m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_PKG_ENERGY;
            m_batch_desc[count].value = 0;
            ++count;

            m_batch_desc[count].device_type = power_domain;
            m_batch_desc[count].device_index = i;
            m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_DRAM_ENERGY;
            m_batch_desc[count].value = 0;
            ++count;

            for (int j = i * freq_domain_per_power_domain; j < (i + 1) * freq_domain_per_power_domain; ++j) {
                m_batch_desc[count].device_type = freq_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_FREQUENCY;
                m_batch_desc[count].value = 0;
                ++count;

                m_batch_desc[count].device_type = freq_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_INST_RETIRED;
                m_batch_desc[count].value = 0;
                ++count;

                m_batch_desc[count].device_type = freq_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_CLK_UNHALTED_CORE;
                m_batch_desc[count].value = 0;
                ++count;

                m_batch_desc[count].device_type = freq_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_CLK_UNHALTED_REF;
                m_batch_desc[count].value = 0;
                ++count;

                m_batch_desc[count].device_type = freq_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_READ_BANDWIDTH;
                m_batch_desc[count].value = 0;
                ++count;
            }
        }
        m_imp->batch_read_signal(m_batch_desc, true);
    }

    size_t RAPLPlatform::capacity(void)
    {
        return m_imp->num_control_domain(m_imp->control_domain()) * (m_imp->num_energy_signal() + m_imp->num_counter_signal());
    }

    void RAPLPlatform::bound(std::map<int, std::pair<double, double> > &bound)
    {
        m_imp->bound(bound);
    }

    void RAPLPlatform::sample(std::vector<struct geopm_msr_message_s> &msr_values)
    {
        int count = 0;
        int signal_index = 0;
        int counter_domain_per_control_domain = m_num_counter_domain / m_num_control_domain;
        int control_domain = m_imp->control_domain();
        struct geopm_time_s time;

        m_imp->batch_read_signal(m_batch_desc, false);
        geopm_time(&time);
        //record per package energy readings
        for (int i = 0; i < m_num_control_domain; i++) {
            msr_values[count].domain_type = control_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_PKG_ENERGY;
            msr_values[count].signal = m_batch_desc[signal_index++].value;

            count++;
            msr_values[count].domain_type = control_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_DRAM_ENERGY;
            msr_values[count].signal = m_batch_desc[signal_index++].value;
            count++;

            msr_values[count].domain_type = control_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_FREQUENCY;
            msr_values[count].signal = accum_freq / counter_domain_per_control_domain;
            count++;

            msr_values[count].domain_type = control_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_INST_RETIRED;
            msr_values[count].signal = accum_inst;
            count++;

            msr_values[count].domain_type = control_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_CLK_UNHALTED_CORE;
            msr_values[count].signal = accum_clk_core;
            count++;

            msr_values[count].domain_type = control_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_CLK_UNHALTED_REF;
            msr_values[count].signal = accum_clk_ref;
            count++;

            msr_values[count].domain_type = control_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_READ_BANDWIDTH;
            msr_values[count].signal = accum_read_bw;
            count++;
        }
    }

    void RAPLPlatform::enforce_policy(uint64_t region_id, IPolicy &policy) const
    {
        int control_type;
        std::vector<double> target(m_num_control_domain);
        policy.target(region_id, GEOPM_CONTROL_TYPE_POWER, target);

        if ((m_control_domain_type == GEOPM_CONTROL_TYPE_POWER) &&
            (m_num_control_domain == (int)target.size())) {
            control_type = GEOPM_TELEMETRY_TYPE_PKG_ENERGY;
            for (int i = 0; i < m_num_package; ++i) {
                m_imp->write_control(m_imp->power_control_domain(), i, control_type, target[i]);
            }
        }
        else {
            if (m_control_domain_type != GEOPM_CONTROL_TYPE_POWER) {
                throw geopm::Exception("RAPLPlatform::enforce_policy: RAPLPlatform Only handles power control domains", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
            if (m_num_control_domain != (int)target.size()) {
                throw geopm::Exception("RAPLPlatform::enforce_policy: Policy size does not match domains of control", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
        }
    }
} //geopm

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
        , m_num_freq_ctl_domain(0)
        , m_num_power_ctl_domain(0)
        , m_num_energy_ctr_domain(0)
        , m_num_pmon_ctr_domain(0)
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
        m_num_freq_ctl_domain = m_imp->num_control_domain(GEOPM_CONTROL_TYPE_FREQUENCY);
        m_num_power_ctl_domain = m_imp->num_control_domain(GEOPM_CONTROL_TYPE_POWER);
        m_num_pmon_ctr_domain = m_imp->num_counter_domain(GEOPM_COUNTER_TYPE_PERF);
        m_num_energy_ctr_domain = m_imp->num_counter_domain(GEOPM_COUNTER_TYPE_ENERGY);
        m_batch_desc.resize(m_num_energy_ctr_domain * m_imp->num_energy_signal() + m_num_pmon_ctr_domain * m_imp->num_counter_signal());

        int count = 0;
        int pmon_domain_per_energy_domain = m_num_pmon_ctr_domain / m_num_energy_ctr_domain;
        int energy_domain = m_imp->counter_domain(GEOPM_COUNTER_TYPE_ENERGY);
        int pmon_domain = m_imp->counter_domain(GEOPM_COUNTER_TYPE_PERF);
        for (int i = 0; i < m_num_energy_ctr_domain; i++) {
            m_batch_desc[count].device_type = energy_domain;
            m_batch_desc[count].device_index = i;
            m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_PKG_ENERGY;
            m_batch_desc[count].value = 0;
            ++count;

            m_batch_desc[count].device_type = energy_domain;
            m_batch_desc[count].device_index = i;
            m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_DRAM_ENERGY;
            m_batch_desc[count].value = 0;
            ++count;

            for (int j = i * pmon_domain_per_energy_domain; j < (i + 1) * pmon_domain_per_energy_domain; ++j) {
                m_batch_desc[count].device_type = pmon_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_FREQUENCY;
                m_batch_desc[count].value = 0;
                ++count;

                m_batch_desc[count].device_type = pmon_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_INST_RETIRED;
                m_batch_desc[count].value = 0;
                ++count;

                m_batch_desc[count].device_type = pmon_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_CLK_UNHALTED_CORE;
                m_batch_desc[count].value = 0;
                ++count;

                m_batch_desc[count].device_type = pmon_domain;
                m_batch_desc[count].device_index = j;
                m_batch_desc[count].signal_type = GEOPM_TELEMETRY_TYPE_CLK_UNHALTED_REF;
                m_batch_desc[count].value = 0;
                ++count;

                m_batch_desc[count].device_type = pmon_domain;
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
        return m_num_energy_ctr_domain * m_imp->num_energy_signal() + m_num_pmon_ctr_domain * m_imp->num_counter_signal();
    }

    void RAPLPlatform::bound(std::map<int, std::pair<double, double> > &bound)
    {
        m_imp->bound(bound);
    }

    void RAPLPlatform::sample(std::vector<struct geopm_msr_message_s> &msr_values)
    {
        int count = 0;
        int signal_index = 0;
        int energy_domain = m_imp->counter_domain(GEOPM_COUNTER_TYPE_ENERGY);
        int pmon_domain = m_imp->counter_domain(GEOPM_COUNTER_TYPE_PERF);
        int pmon_domain_per_energy_domain = m_num_pmon_ctr_domain / m_num_energy_ctr_domain;
        struct geopm_time_s time;

        m_imp->batch_read_signal(m_batch_desc, false);
        geopm_time(&time);
        //record per package energy readings
        for (int i = 0; i < m_num_energy_ctr_domain; i++) {
            msr_values[count].domain_type = energy_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_PKG_ENERGY;
            msr_values[count].signal = m_batch_desc[signal_index++].value;

            count++;
            msr_values[count].domain_type = energy_domain;
            msr_values[count].domain_index = i;
            msr_values[count].timestamp = time;
            msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_DRAM_ENERGY;
            msr_values[count].signal = m_batch_desc[signal_index++].value;
            count++;

            for (int j = i * pmon_domain_per_energy_domain; j < (i + 1) * pmon_domain_per_energy_domain; ++j) {
                msr_values[count].domain_type = pmon_domain;
                msr_values[count].domain_index = j;
                msr_values[count].timestamp = time;
                msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_FREQUENCY;
                msr_values[count].signal = m_batch_desc[signal_index++].value;
                count++;

                msr_values[count].domain_type = pmon_domain;
                msr_values[count].domain_index = j;
                msr_values[count].timestamp = time;
                msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_INST_RETIRED;
                msr_values[count].signal = m_batch_desc[signal_index++].value;
                count++;

                msr_values[count].domain_type = pmon_domain;
                msr_values[count].domain_index = j;
                msr_values[count].timestamp = time;
                msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_CLK_UNHALTED_CORE;
                msr_values[count].signal = m_batch_desc[signal_index++].value;
                count++;

                msr_values[count].domain_type = pmon_domain;
                msr_values[count].domain_index = j;
                msr_values[count].timestamp = time;
                msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_CLK_UNHALTED_REF;
                msr_values[count].signal = m_batch_desc[signal_index++].value;
                count++;

                msr_values[count].domain_type = pmon_domain;
                msr_values[count].domain_index = j;
                msr_values[count].timestamp = time;
                msr_values[count].signal_type = GEOPM_TELEMETRY_TYPE_READ_BANDWIDTH;
                msr_values[count].signal = m_batch_desc[signal_index++].value;
                count++;
            }
        }
    }

    void RAPLPlatform::enforce_policy(uint64_t region_id, IPolicy &policy) const
    {
        int control_type;
        std::vector<double> pwr_target(m_num_power_ctl_domain);
        std::vector<double> freq_target(m_num_freq_ctl_domain);
        policy.target(region_id, GEOPM_CONTROL_TYPE_POWER, pwr_target);
        policy.target(region_id, GEOPM_CONTROL_TYPE_FREQUENCY, freq_target);

        if ((m_num_power_ctl_domain == (int)pwr_target.size()) &&
            (m_num_freq_ctl_domain == (int)freq_target.size())) {
            if (pwr_target[0] != GEOPM_TARGET_INVALID) {
                int i = 0;
                for (auto it = pwr_target.begin(); it != pwr_target.end(); ++it) {
                    m_imp->write_control(m_imp->control_domain(GEOPM_CONTROL_TYPE_POWER), i, GEOPM_CONTROL_TYPE_POWER, (*it));
                    ++i;
                }
            }
            if (freq_target[0] != GEOPM_TARGET_INVALID) {
                int i = 0;
                for (auto it = freq_target.begin(); it != freq_target.end(); ++it) {
                    m_imp->write_control(m_imp->control_domain(GEOPM_CONTROL_TYPE_FREQUENCY), i, GEOPM_CONTROL_TYPE_FREQUENCY, (*it));
                    ++i;
                }
            }
        }
        else {
            throw geopm::Exception("RAPLPlatform::enforce_policy: Policy size does not match domains of control", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }
} //geopm

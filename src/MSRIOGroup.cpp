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

#include <cpuid.h>

#include "MSRIOGroup.hpp"
#include "MSRIO.hpp"

namespace geopm
{
    const MSR *msr_knl(size_t &num_msr);
    const MSR *msr_hsx(size_t &num_msr);
    const MSR *msr_snb(size_t &num_msr);

    MSRIOGroup::MSRIOGroup()
        : m_num_cpu(geopm_sched_num_cpu())
        , m_msrio(new MSRIO())
    {
        init_msr();
    }

    virtual MSRIOGroup::~MSRIOGroup()
    {

    }

    bool MSRIOGroup::is_valid_signal(const std::string &signal_name)
    {
        return false;
    }

    bool MSRIOGroup::is_valid_control(const std::string &control_name)
    {
        return false;
    }

    int MSRIOGroup::push_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        return -1;
    }

    int MSRIOGroup::push_control(const std::string &control_name, int domain_type, int domain_idx)
    {
        return -1;
    }

    void MSRIOGroup::read_batch(void)
    {

    }

    void MSRIOGroup::write_batch(void)
    {

    }

    void MSRIOGroup::sample(std::vector<double> &signal)
    {

    }

    double MSRIOGroup::sample(int batch_idx)
    {
        return NAN;
    }

    void MSRIOGroup::adjust(const std::vector<double> &setting)
    {

    }

    void MSRIOGroup::adjust(int batch_idx, double setting)
    {

    }

    double MSRIOGroup::read_signal(const std::string &signal_name, int domain_type, int domain_idx)
    {
        return NAN;
    }

    void MSRIOGroup::write_control(const std::string &control_name, int domain_type, int domain_idx, double setting)
    {

    }

    int MSRIOGroup::cpuid(void) const
    {
        uint32_t key = 1; //processor features
        uint32_t proc_info = 0;
        uint32_t model;
        uint32_t family;
        uint32_t ext_model;
        uint32_t ext_family;
        uint32_t ebx, ecx, edx;
        const uint32_t model_mask = 0xF0;
        const uint32_t family_mask = 0xF00;
        const uint32_t extended_model_mask = 0xF0000;
        const uint32_t extended_family_mask = 0xFF00000;

        __get_cpuid(key, &proc_info, &ebx, &ecx, &edx);

        model = (proc_info & model_mask) >> 4;
        family = (proc_info & family_mask) >> 8;
        ext_model = (proc_info & extended_model_mask) >> 16;
        ext_family = (proc_info & extended_family_mask) >> 20;

        if (family == 6) {
            model+=(ext_model << 4);
        }
        else if (family == 15) {
            model+=(ext_model << 4);
            family+=ext_family;
        }

        return ((family << 8) + model);
    }

    void MSRIOGroup::init_msr(void)
    {
        size_t num_msr = 0;
        const MSR *msr_arr = init_msr_arr(cpuid(), num_msr);
        for (const MSR *msr_ptr = msr_arr;
             msr_ptr != msr_arr + num_msr;
             ++msr_ptr) {
            m_name_msr_map.insert(std::pair<std::string, const IMSR *>(msr_ptr->name(), msr_ptr));
            for (int idx = 0; idx < msr_ptr->num_signal(); idx++) {
                register_msr_signal(msr_ptr->name() + ":" + msr_ptr->signal_name(idx));
            }
            for (int idx = 0; idx < msr_ptr->num_control(); idx++) {
                register_msr_control(msr_ptr->name() + ":" + msr_ptr->control_name(idx));
            }
        }
    }

    void MSRIOGroup::register_msr_signal(const std::string &signal_name)
    {
        size_t colon_pos = signal_name.find(':');
        if (colon_pos == std::string::npos) {
            throw Exception("MSRIOGroup::register_msr_signal(): signal_name must be of the form \"msr_name:field_name\"",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        std::vector<std::string> msr_name_vec({signal_name.substr(0, colon_pos)});
        std::vector<std::string> field_name_vec({signal_name.substr(colon_pos + 1)});
        register_msr_signal(signal_name, msr_name_vec, field_name_vec);
    }


    void MSRIOGroup::register_msr_signal(const std::string &signal_name,
                                         const std::vector<std::string> &msr_name,
                                         const std::vector<std::string> &field_name)
    {
        // Assert that msr_name and field_name are the same size.
        if (msr_name.size() != field_name.size()) {
            throw Exception("MSRIOGroup::register_msr_signal(): signal_name vector length does not match msr_name",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        // Insert the signal name with an empty vector into the map
        auto ins_ret = m_name_cpu_signal_map.insert(std::pair<std::string, std::vector<ISignal *> >(signal_name, {}));
        // Get reference to the per-cpu signal vector
        std::vector <ISignal *> &cpu_signal = (*(ins_ret.first)).second;
        // Check to see if the signal name has already been registered
        if (!ins_ret.second) {
            /* delete previous signals */
            for (auto &cs : cpu_signal) {
                delete cs;
            }
        }
        cpu_signal.resize(m_num_cpu, NULL);
        int num_field = field_name.size();

        std::vector<struct IMSRSignal::m_signal_config_s> signal_config(num_field);
        int field_idx = 0;
        for (auto &sc : signal_config) {
            auto name_msr_it = m_name_msr_map.find(msr_name[field_idx]);
            if (name_msr_it == m_name_msr_map.end()) {
                throw Exception("MSRIOGroup::register_msr_signal(): msr_name could not be found",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
            sc.msr_obj = name_msr_it->second;
            sc.signal_idx = sc.msr_obj->signal_index(field_name[field_idx]);
            if (sc.signal_idx == -1) {
                throw Exception("MSRIOGroup::register_msr_signal(): field_name could not be found",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
            ++field_idx;
        }

        for (int cpu_idx = 0; cpu_idx < m_num_cpu; ++cpu_idx) {
            for (auto &sc : signal_config) {
                sc.cpu_idx = cpu_idx;
            }
            cpu_signal[cpu_idx] = new MSRSignal(signal_config, signal_name);
        }
    }

    void MSRIOGroup::register_msr_control(const std::string &control_name)
    {
        size_t colon_pos = control_name.find(':');
        if (colon_pos == std::string::npos) {
            throw Exception("MSRIOGroup::register_msr_control(): control_name must be of the form \"msr_name:field_name\"",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        std::vector<std::string> msr_name_vec({control_name.substr(0, colon_pos)});
        std::vector<std::string> field_name_vec({control_name.substr(colon_pos + 1)});
        register_msr_control(control_name, msr_name_vec, field_name_vec);
    }

    void MSRIOGroup::register_msr_control(const std::string &control_name,
                                          const std::vector<std::string> &msr_name,
                                          const std::vector<std::string> &field_name)
    {
        // Assert that msr_name and field_name are the same size.
        if (msr_name.size() != field_name.size()) {
            throw Exception("MSRIOGroup::register_msr_control(): control_name vector length does not match msr_name",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        // Insert the control name with an empty vector into the map
        auto ins_ret = m_name_cpu_control_map.insert(std::pair<std::string, std::vector<IControl *> >(control_name, {}));
        // Get reference to the per-cpu control vector
        std::vector <IControl *> &cpu_control = (*(ins_ret.first)).second;
        // Check to see if the control name has already been registered
        if (!ins_ret.second) {
            /* delete previous controls */
            for (auto &cc : cpu_control) {
                delete cc;
            }
        }
        cpu_control.resize(m_num_cpu, NULL);
        int num_field = field_name.size();

        std::vector<struct IMSRControl::m_control_config_s> control_config(num_field);
        int field_idx = 0;
        for (auto &sc : control_config) {
            auto name_msr_it = m_name_msr_map.find(msr_name[field_idx]);
            if (name_msr_it == m_name_msr_map.end()) {
                throw Exception("MSRIOGroup::register_msr_control(): msr_name could not be found",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
            sc.msr_obj = name_msr_it->second;
            sc.control_idx = sc.msr_obj->control_index(field_name[field_idx]);
            if (sc.control_idx == -1) {
                throw Exception("MSRIOGroup::register_msr_control(): field_name could not be found",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
            ++field_idx;
        }

        for (int cpu_idx = 0; cpu_idx < m_num_cpu; ++cpu_idx) {
            for (auto &sc : control_config) {
                sc.cpu_idx = cpu_idx;
            }
            cpu_control[cpu_idx] = new MSRControl(control_config, control_name);
        }
    }

    static const MSR *MSRIOGroup::init_msr_arr(size_t &arr_size)
    {
        const MSR *msr_arr = NULL;
        num_msr = 0;
        switch (cpu_id) {
            case MSRIOGroup::M_CPUID_KNL:
                msr_arr = msr_knl(num_msr);
                break;
            case MSRIOGroup::M_CPUID_HSX:
            case MSRIOGroup::M_CPUID_BDX:
                msr_arr = msr_hsx(num_msr);
                break;
            case MSRIOGroup::M_CPUID_SNB:
            case MSRIOGroup::M_CPUID_IVT:
                msr_arr = msr_snb(num_msr);
                break;
            default:
                throw Exception("MSRIOGroup: Unsupported CPUID",
                                GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        return msr_arr;
    }
}

#endif

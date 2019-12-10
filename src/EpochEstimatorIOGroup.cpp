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

#include <cmath>

#include "geopm.h"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "EpochEstimatorIOGroup.hpp"
#include "Agg.hpp"
#include "Helper.hpp"

namespace geopm
{
    class EpochEstimatorAtom
    {
        public:
            EpochEstimatorAtom();
            virtual ~EpochEstimatorAtom() = default;
            void cpu_idx(int idx);
            int epoch_count(void);
        private:
            int m_all_reduce_count;
            int m_region_hash_idx;
            uint64_t m_region_hash;
    };


    EpochEstimatorAtom::EpochEstimatorAtom()
        : m_all_reduce_count(0)
        , m_region_hash_idx(-1)
        , m_region_hash(GEOPM_REGION_HASH_INVALID)
    {

    }

    void EpochEstimatorAtom::cpu_idx(int idx)
    {
        m_region_hash_idx = platform_io().push_signal("REGION_HASH", GEOPM_DOMAIN_CPU, idx);
    }

    int EpochEstimatorAtom::epoch_count(void)
    {
        uint64_t region_hash = platform_io().sample(m_region_hash_idx);
        if (region_hash == 0xd94e328ULL &&
            region_hash != m_region_hash) {
            ++m_all_reduce_count;
        }
        m_region_hash = region_hash;
        return m_all_reduce_count;
    }

    EpochEstimatorIOGroup::EpochEstimatorIOGroup()
        : m_atoms(platform_topo().num_domain(GEOPM_DOMAIN_CPU))
    {

    }

    EpochEstimatorIOGroup::~EpochEstimatorIOGroup()
    {

    }

    std::set<std::string> EpochEstimatorIOGroup::signal_names(void) const
    {
        return {"EPOCH_COUNT"};
    }

    std::set<std::string> EpochEstimatorIOGroup::control_names(void) const
    {
        return {};
    }

    bool EpochEstimatorIOGroup::is_valid_signal(const std::string &signal_name) const
    {
        return signal_name == "EPOCH_COUNT";
    }

    bool EpochEstimatorIOGroup::is_valid_control(const std::string &control_name) const
    {
        return false;
    }

    int EpochEstimatorIOGroup::signal_domain_type(const std::string &signal_name) const
    {
        return GEOPM_DOMAIN_CPU;
    }

    int EpochEstimatorIOGroup::control_domain_type(const std::string &control_name) const
    {
        return GEOPM_DOMAIN_INVALID;
    }

    int EpochEstimatorIOGroup::push_signal(const std::string &signal_name,
                                    int domain_type,
                                    int domain_idx)
    {
        if (!is_valid_signal(signal_name) ||
            domain_type != GEOPM_DOMAIN_CPU) {
            throw Exception("EpochEstimatorIOGroup::push_signal(): invalid signal_name or domain_type",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        m_atoms.at(domain_idx).cpu_idx(domain_idx);
        return domain_idx;
    }

    int EpochEstimatorIOGroup::push_control(const std::string &control_name,
                                     int domain_type,
                                     int domain_idx)
    {
        throw Exception("EpochEstimatorIOGroup::push_control(): Does not support any controls",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        return -1;
    }

    void EpochEstimatorIOGroup::read_batch(void)
    {

    }

    void EpochEstimatorIOGroup::write_batch(void)
    {

    }

    double EpochEstimatorIOGroup::sample(int sample_idx)
    {
        return m_atoms.at(sample_idx).epoch_count();
    }

    void EpochEstimatorIOGroup::adjust(int control_idx,
                                double setting)
    {

    }

    double EpochEstimatorIOGroup::read_signal(const std::string &signal_name,
                                       int domain_type,
                                       int domain_idx)
    {
        return NAN;
    }

    void EpochEstimatorIOGroup::write_control(const std::string &control_name,
                                       int domain_type,
                                       int domain_idx,
                                       double setting)
    {

    }

    void EpochEstimatorIOGroup::save_control(void)
    {

    }

    void EpochEstimatorIOGroup::restore_control(void)
    {

    }

    std::function<double(const std::vector<double> &)> EpochEstimatorIOGroup::agg_function(const std::string &signal_name) const
    {
        return Agg::min;
    }

    std::function<std::string(double)> EpochEstimatorIOGroup::format_function(const std::string &signal_name) const
    {
        return string_format_integer;
    }

    std::string EpochEstimatorIOGroup::signal_description(const std::string &signal_name) const
    {
        return "";
    }

    std::string EpochEstimatorIOGroup::control_description(const std::string &control_name) const
    {
        return "";
    }

    std::string EpochEstimatorIOGroup::plugin_name(void)
    {
        return "EpochEstimatorIOGroup";
    }

    std::unique_ptr<IOGroup> EpochEstimatorIOGroup::make_plugin(void)
    {
        return geopm::make_unique<EpochEstimatorIOGroup>();
    }

}

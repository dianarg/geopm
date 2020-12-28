/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
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

#ifndef RECORDACCOUNT_HPP_INCLUDE
#define RECORDACCOUNT_HPP_INCLUDE

#include <cstdint>

#include <map>

namespace geopm
{
    // TODO: confirm name or think of a better one.  Or should this
    // just be a part of ApplicationSampler?

    /// @brief Class responsible for reading records from the
    ///        ApplicationSampler and calculating the per-process
    ///        runtimes within each region.
    class RecordAccount
    {
        public:
            virtual ~RecordAccount() = default;
            /// @brief Gets the latest set of records from
            ///        ApplicationSampler.
            virtual void update(void) = 0;
            /// @brief Returns the total time spent in a region of
            ///        each process.  Processes that never entered the
            ///        region are not included.
            ///
            /// @param [in] region_hash Hash of the region.
            virtual std::map<int, double> get_process_region_runtime(uint64_t region_hash) const = 0;
            /// @brief Returns the average total time spent in a
            ///        region across all processes that were ever
            ///        active in that region.
            ///
            /// @param [in] region_hash Hash of the region.
            virtual double get_process_region_runtime_average(uint64_t region_hash) const = 0;
            /// @brief Returns the number of entries into a region of
            ///        each process.  Processes that never entered the
            ///        region are not included.
            ///
            /// @param [in] region_hash Hash of the region.
            virtual std::map<int, int> get_process_region_count(uint64_t region_hash) const = 0;
            /// @brief Returns the average number of entries into a
            ///        region across all processes that were ever
            ///        active in that region.
            ///
            /// @param [in] region_hash Hash of the region.
            virtual double get_process_region_count_average(uint64_t region_hash) const = 0;
    };

    class ApplicationSampler;
    class SumAccumlator;

    class RecordAccountImp : public RecordAccount
    {
        public:
            RecordAccountImp();
            RecordAccountImp(ApplicationSampler &sampler);
            virtual ~RecordAccountImp() = default;
            void update(void) override;
            std::map<int, double> get_process_region_runtime(uint64_t region_hash) const override;
            double get_process_region_runtime_average(uint64_t region_hash) const override;
            std::map<int, int> get_process_region_count(uint64_t region_hash) const override;
            double get_process_region_count_average(uint64_t region_hash) const override;
        private:
            ApplicationSampler &m_app_sampler;

            // Records will be coming in sorted by process.  An
            // optimization might be to keep an iterator around
            // pointing to the most recent process's map.  The lookup
            // by region hash will happen less frequently but requires
            // iteration over all the process maps.  Build a cache and
            // invalidate it if update() is called.
            struct region_info_s {
                    std::unique_ptr<SumAccumlator> runtime;
                    std::unique_ptr<SumAccumlator> count;

                    //double total_runtime;
                    //int total_count;
                    //double last_entry_time;
            };
            std::map<int, std::map<uint64_t, region_info_s> > m_region_info;


            // TODO: this can just use new Accumulator for Sum ?

            // Mapping from process and region hash to last entry and total runtime
            //std::map<std::pair<int, uint64_t>, std::pair<double, double> > m_region_runtimes;
            // Mapping from process and region hash to count
            //std::map<std::pair<int, uint64_t>, int > m_region_counts;

    };
}

#endif

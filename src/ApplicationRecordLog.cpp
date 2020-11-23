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

#include "ApplicationRecordLog.hpp"
#include "SharedMemory.hpp"
#include "Exception.hpp"
#include "Helper.hpp"

namespace geopm
{
    std::unique_ptr<ApplicationRecordLog> ApplicationRecordLog::make_unique(std::shared_ptr<SharedMemory> shmem)
    {
        return geopm::make_unique<ApplicationRecordLogImp>(shmem);
    }

    size_t ApplicationRecordLog::buffer_size(void)
    {
        return sizeof(m_layout_s);
    }

    ApplicationRecordLogImp::ApplicationRecordLogImp(std::shared_ptr<SharedMemory> shmem)
        : m_process(-1)
        , m_shmem(shmem)
        , m_time_zero({{0, 0}})
        , m_is_setup(false)
        , m_epoch_count(0)
    {

    }

    void ApplicationRecordLogImp::set_process(int process)
    {
        if (m_is_setup) {
            throw Exception("ApplicationRecordLog::set_process() called after process has been used",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        m_process = process;
    }

    void ApplicationRecordLogImp::set_time_zero(const geopm_time_s &time)
    {
        if (m_is_setup) {
            throw Exception("ApplicationRecordLog::set_time_zero() called after time zero has been used",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        m_time_zero = time;
    }

    void ApplicationRecordLogImp::enter(uint64_t hash, const geopm_time_s &time)
    {
        check_setup();
        std::unique_ptr<SharedMemoryScopedLock> lock = m_shmem->get_scoped_lock();
        m_layout_s &layout = *((m_layout_s *)(m_shmem->pointer()));
        check_reset(layout);

        record_s enter_record = {
           .time = geopm_time_diff(&m_time_zero, &time),
           .process = m_process,
           .event = EVENT_REGION_ENTRY,
           .signal = hash,
        };
        append_record(layout, enter_record);
    }
    void ApplicationRecordLogImp::exit(uint64_t hash, const geopm_time_s &time)
    {
        check_setup();
        std::unique_ptr<SharedMemoryScopedLock> lock = m_shmem->get_scoped_lock();
        m_layout_s &layout = *((m_layout_s *)(m_shmem->pointer()));
        check_reset(layout);

        record_s exit_record = {
           .time = geopm_time_diff(&m_time_zero, &time),
           .process = m_process,
           .event = EVENT_REGION_EXIT,
           .signal = hash,
        };
        append_record(layout, exit_record);
    }

    void ApplicationRecordLogImp::epoch(const geopm_time_s &time)
    {
        check_setup();
        std::unique_ptr<SharedMemoryScopedLock> lock = m_shmem->get_scoped_lock();
        m_layout_s &layout = *((m_layout_s *)(m_shmem->pointer()));
        check_reset(layout);

        ++m_epoch_count;
        record_s epoch_record = {
           .time = geopm_time_diff(&m_time_zero, &time),
           .process = m_process,
           .event = EVENT_EPOCH_COUNT,
           .signal = m_epoch_count,
        };
        append_record(layout, epoch_record);
    }

    void ApplicationRecordLogImp::dump(std::vector<record_s> &records,
                                       std::vector<short_region_s> &short_regions)
    {
        std::unique_ptr<SharedMemoryScopedLock> lock = m_shmem->get_scoped_lock();
        m_layout_s &layout = *((m_layout_s *)(m_shmem->pointer()));
        records.resize(layout.num_record);
        std::copy(layout.record_table, layout.record_table + layout.num_record, records.begin());
    }

    void ApplicationRecordLogImp::check_setup(void)
    {
        if (m_is_setup) {
            return;
        }

        if (m_process == -1) {
            throw Exception("ApplicationRecordLog: set_process() must be called prior to calling enter(), exit() or epoch()",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }

        geopm_time_s zero = {{0, 0}};
        if (geopm_time_diff(&m_time_zero, &zero) == 0.0) {
            throw Exception("ApplicationRecordLog: set_time_zero() must be called prior to calling enter(), exit() or epoch()",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }

        m_is_setup = true;
    }

    void ApplicationRecordLogImp::check_reset(const m_layout_s &layout)
    {
        if (layout.num_record == 0) {
            m_hash_record_map.clear();
        }
    }


    int ApplicationRecordLogImp::append_record(m_layout_s &layout, const record_s &record)
    {
        int record_idx = layout.num_record;
        // Don't overrun the buffer
        if (record_idx < M_MAX_RECORD) {
            layout.record_table[record_idx] = record;
            ++(layout.num_record);
        }
        return record_idx;
    }
}

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


namespace geopm
{
    void ApplicationRecordLog::check_reset(void)
    {
        if (m_shmem->num_record == 0)
        {
            m_hash_record_map.clear();
        }
    }


    void ApplicationRecordLog::set_process(int process)
    {

    }

    /// @brief Get the index into the short_table for a
    /// specific hash.  If the hash has not been observed
    /// since the last dump() call a new element in the
    /// table is initialized
    void ApplicationRecordLog::enter(uint64_t hash, const geopm_time_s &time)
    {
        auto emplace_pair = m_hash_record_map.emplace(hash, m_shmem->num_enter);
        bool is_new  = emplace_pair.second;
        int short_idx = emplace_pair.first->second;

        // Region with hash has not been entered since last dump
        if (short_idx < M_MAX_ENTER) {
            if (is_new) {
                m_shmem->short_table[short_idx] = {
                    .record_idx = result,
                    .hash = hash,
                    .enter_time = time,
                    .num_complete = 0,
                    .total_time = 0.0,
                };
                ++(m_shmem->num_enter);
                record_s enter_record = {
                    .time = time,
                    .process = m_process,
                    .event = EVENT_REGION_ENTRY,
                    .signal = hash,
                };
                append_record(enter_record);
            }
            else {
                m_shmem->short_table[short_idx].enter_time = time;
            }
        }
        else {
            throw Exception("Table overflow");
        }
        return result;
    }
}

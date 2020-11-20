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
    class ApplicationRecordLog
    {
        public:
            std::unique_ptr<ApplicationRecordLog> record_log(std::shared_ptr<SharedMemory> shmem);
            std::unique_ptr<ApplicationRecordLog> record_log(std::shared_ptr<SharedMemoryUser> shmem);
            void set_process(int process) = 0;
            void set_zero_time(const geopm_time_s &time) = 0;
            virtual void enter(uint64_t hash, const geopm_time_s &time) = 0;
            virtual void exit(uint64_t hash, const geopm_time_s &time) = 0;
            virtual void epoch(const geopm_time_s &time) = 0;
            virtual void dump(std::vector<record_s> &records,
                              std::vector<short_region_s> &short_regions) = 0;
            static size_t buffer_size(void);
        private:
            RecordLog() = default;
            virtual ~RecordLog() = default;
    }

    class ApplicationRecordLogImp
    {
        public:
            RecordLogImp(std::shared_ptr<SharedMemory> shmem);
            RecordLogImp(std::shared_ptr<SharedMemoryUser> shmem);
            void set_process(int process) override;
            void set_zero_time(const geopm_time_s &time) override;
            void enter(uint64_t hash, const geopm_time_s &time) override;
            void exit(uint64_t hash, const geopm_time_s &time) override;
            void epoch(const geopm_time_s &time) override;
            void dump(std::vector<record_s> &records,
                      std::vector<short_region_s> &short_regions) override;
        private:
            void check_reset(void);
            void *pointer(void) const;
            std::unique_ptr<SharedMemoryScopedLock> get_scoped_lock(void);

            struct m_short_el_s {
                int record_idx;
                uint64_t hash;
                geopm_time_s enter_time;
                int num_complete;
                double total_time;
            };
            static constexpr size_t M_MAX_ENTER = 1024;
            static constexpr size_t M_MAX_RECORD = 1024;
            struct m_layout_s {
                size_t num_record;
                struct m_record_s record_table[M_MAX_RECORD];
                size_t num_enter;
                struct m_short_el_s short_table[M_MAX_ENTER];
            };
            int m_process;
            std::shared_ptr<SharedMemory> m_shmem;
            std::shared_ptr<SharedMemoryUser> m_shmem_user;
            std::map<uint64_t, int> m_hash_record_map;
    };
}

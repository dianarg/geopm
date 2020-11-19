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
#include "config.h"

#include "Reporter.hpp"

#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#ifdef GEOPM_HAS_XMMINTRIN
#include <xmmintrin.h>
#endif

#include <sstream>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>

#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "RegionAggregatorImp.hpp"
#include "ApplicationIO.hpp"
#include "Comm.hpp"
#include "TreeComm.hpp"
#include "Exception.hpp"
#include "Helper.hpp"
#include "geopm.h"
#include "geopm_hash.h"
#include "geopm_version.h"
#include "Environment.hpp"
#include "geopm_debug.hpp"

namespace geopm
{
    ReporterImp::ReporterImp(const std::string &start_time,
                             const std::string &report_name,
                             PlatformIO &platform_io,
                             const PlatformTopo &platform_topo,
                             int rank)
        : ReporterImp(start_time,
                      report_name,
                      platform_io,
                      platform_topo,
                      rank,
                      std::unique_ptr<RegionAggregator>(new RegionAggregatorImp),
                      environment().report_signals(),
                      environment().policy(),
                      environment().do_endpoint())
    {

    }

    ReporterImp::ReporterImp(const std::string &start_time,
                             const std::string &report_name,
                             PlatformIO &platform_io,
                             const PlatformTopo &platform_topo,
                             int rank,
                             std::unique_ptr<RegionAggregator> agg,
                             const std::string &env_signals,
                             const std::string &policy_path,
                             bool do_endpoint)
        : m_start_time(start_time)
        , m_report_name(report_name)
        , m_platform_io(platform_io)
        , m_platform_topo(platform_topo)
        , m_region_agg(std::move(agg))
        , m_rank(rank)
        , m_region_bulk_runtime_idx(-1)
        , m_energy_pkg_idx(-1)
        , m_energy_dram_idx(-1)
        , m_clk_core_idx(-1)
        , m_clk_ref_idx(-1)
        , m_env_signals(env_signals)
        , m_policy_path(policy_path)
        , m_do_endpoint(do_endpoint)
        , m_app_energy_pkg_idx(-1)
        , m_app_energy_dram_idx(-1)
        , m_app_time_signal_idx(-1)
        , m_start_energy_pkg(NAN)
        , m_start_energy_dram(NAN)
        , m_start_time_signal(NAN)
    {

    }

    struct signal_info
    {
        std::string name;
        int domain_type;
        int domain_idx;
    };

    // TODO: shared code with trace extensions env_signals() and env_domains()
    static std::vector<signal_info> parse_env_signals(const PlatformTopo &topo,
                                                      const std::string &env_signals)
    {
        std::vector<signal_info> result;
        for (const std::string &env_sig : string_split(env_signals, ",")) {
            std::vector<std::string> signal_name_domain = string_split(env_sig, "@");
            if (signal_name_domain.size() == 2) {
                std::string signal_name = signal_name_domain[0];
                int domain_type = PlatformTopo::domain_name_to_type(signal_name_domain[1]);
                for (int domain_idx = 0; domain_idx < topo.num_domain(domain_type); ++domain_idx) {
                    result.push_back({signal_name, domain_type, domain_idx});
                }
            }
            else if (signal_name_domain.size() == 1) {
                std::string signal_name = signal_name_domain[0];
                result.push_back({signal_name, GEOPM_DOMAIN_BOARD, 0});
            }
            else {
                throw Exception("ReporterImp::parse_env_signals(): Environment report extension contains signals with multiple \"@\" characters.",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
        }

        return result;
    }

    void ReporterImp::init(void)
    {
        m_region_bulk_runtime_idx = m_region_agg->push_signal_total("TIME", GEOPM_DOMAIN_BOARD, 0);
        m_energy_pkg_idx = m_region_agg->push_signal_total("ENERGY_PACKAGE", GEOPM_DOMAIN_BOARD, 0);
        m_energy_dram_idx = m_region_agg->push_signal_total("ENERGY_DRAM", GEOPM_DOMAIN_BOARD, 0);
        m_clk_core_idx = m_region_agg->push_signal_total("CYCLES_THREAD", GEOPM_DOMAIN_BOARD, 0);
        m_clk_ref_idx = m_region_agg->push_signal_total("CYCLES_REFERENCE", GEOPM_DOMAIN_BOARD, 0);

        m_app_time_signal_idx = m_platform_io.push_signal("TIME", GEOPM_DOMAIN_BOARD, 0);
        m_app_energy_pkg_idx = m_platform_io.push_signal("ENERGY_PACKAGE", GEOPM_DOMAIN_BOARD, 0);
        m_app_energy_dram_idx = m_platform_io.push_signal("ENERGY_DRAM", GEOPM_DOMAIN_BOARD, 0);

        std::vector<signal_info> region_extensions = parse_env_signals(m_platform_topo, m_env_signals);
        for (const auto &sig : region_extensions) {
            int agg_idx = m_region_agg->push_signal_total(sig.name, sig.domain_type, sig.domain_idx);
            std::string key_name = sig.name;
            if (sig.domain_type != GEOPM_DOMAIN_BOARD) {
                key_name += "@" + PlatformTopo::domain_type_to_name(sig.domain_type);
                key_name += "-" + std::to_string(sig.domain_idx);
            }
            m_env_signal_name_idx.emplace_back(key_name, agg_idx);
        }

        if (!m_rank) {
            // check if report file can be created
            if (!m_report_name.empty()) {
                std::ofstream test_open(m_report_name);
                if (!test_open.good()) {
                    std::cerr << "Warning: <geopm> Unable to open report file '" << m_report_name
                              << "' for writing: " << strerror(errno) << std::endl;
                }
                std::remove(m_report_name.c_str());
            }
        }
        m_region_agg->init();
    }

    void ReporterImp::update()
    {
        if (isnan(m_start_energy_pkg)) {
            m_start_energy_pkg = m_platform_io.sample(m_app_energy_pkg_idx);
            m_start_energy_dram = m_platform_io.sample(m_app_energy_dram_idx);
            m_start_time_signal = m_platform_io.sample(m_app_time_signal_idx);
        }

        m_region_agg->read_batch();
    }

    void ReporterImp::format_header(std::ostream &report,
                                    const std::string &profile_name,
                                    const std::string &agent_name,
                                    const std::vector<std::pair<std::string, std::string> > &agent_report_header) const
    {
        report << "##### geopm " << geopm_version() << " #####" << std::endl;
        report << "Start Time: " << m_start_time << std::endl;
        report << "Profile: " << profile_name << std::endl;
        report << "Agent: " << agent_name << std::endl;
        std::string policy_str = "{}";
        if (m_do_endpoint) {
            policy_str = "DYNAMIC";
        }
        else if (m_policy_path.size() > 0) {
            try {
                policy_str = read_file(m_policy_path);
            }
            catch(...) {
                policy_str = m_policy_path;
            }
        }
        report << "Policy: " << policy_str << std::endl;
        for (const auto &kv : agent_report_header) {
            report << kv.first << ": " << kv.second << std::endl;
        }
    }

    void ReporterImp::format_all_regions(std::ostream &report,
                                         const ApplicationIO &application_io,
                                         const std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > &agent_region_report) const
    {
        // vector of region data, in descending order by runtime
        struct region_info {
            std::string name;
            uint64_t hash;
            double per_rank_avg_runtime;
            int count;
        };
        std::vector<region_info> region_ordered;
        auto region_name_set = application_io.region_name_set();
        for (const auto &region : region_name_set) {
            uint64_t region_hash = geopm_crc32_str(region.c_str());
            int count = application_io.total_count(region_hash);
            if (count > 0) {
                region_ordered.push_back({region,
                                          region_hash,
                                          application_io.total_region_runtime(region_hash),
                                          count});
            }
        }
        // sort based on averge runtime, descending
        std::sort(region_ordered.begin(), region_ordered.end(),
                  [] (const region_info &a,
                      const region_info &b) -> bool {
                      return a.per_rank_avg_runtime > b.per_rank_avg_runtime;
                  });
        // Add unmarked and epoch at the end
        // Note here we map the private region id notion of
        // GEOPM_REGION_HASH_UNMARKED to pubilc GEOPM_REGION_HASH_UNMARKED.
        region_ordered.push_back({"unmarked-region",
                                  GEOPM_REGION_HASH_UNMARKED,
                                  application_io.total_region_runtime(GEOPM_REGION_HASH_UNMARKED),
                                  0});
        for (const auto &region : region_ordered) {
#ifdef GEOPM_DEBUG
            if (GEOPM_REGION_HASH_INVALID == region.hash) {
                throw Exception("ReporterImp::generate(): Invalid hash value detected.",
                                GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
            }
#endif
            std::vector<std::pair<std::string, std::string> > agent_region_ext;
            if (agent_region_report.find(region.hash) != agent_region_report.end()) {
                agent_region_ext = agent_region_report.at(region.hash);
            }
            format_region(report, application_io, region.name, region.hash,
                          region.per_rank_avg_runtime, region.count,
                          agent_region_ext);
        }
    }

    void ReporterImp::format_region(std::ostream &report,
                                    const ApplicationIO &application_io,
                                    const std::string &region_name,
                                    uint64_t region_hash,
                                    double avg_runtime,
                                    int count,
                                    const std::vector<std::pair<std::string, std::string> > &agent_region_report) const
    {
        report << "Region " << region_name << " (0x" << std::hex
               << std::setfill('0') << std::setw(16)
               << region_hash << std::dec << "):"
               << std::setfill('\0') << std::setw(0)
               << std::endl;
        double sync_rt = m_region_agg->sample_total(m_region_bulk_runtime_idx, region_hash);
        double package_energy = m_region_agg->sample_total(m_energy_pkg_idx, region_hash);
        double power = sync_rt == 0 ? 0 : package_energy / sync_rt;
        report << "    runtime (sec): " << avg_runtime << std::endl;
        report << "    sync-runtime (sec): " << sync_rt << std::endl;
        report << "    package-energy (joules): " << package_energy << std::endl;
        report << "    dram-energy (joules): " << m_region_agg->sample_total(m_energy_dram_idx, region_hash) << std::endl;
        report << "    power (watts): " << power << std::endl;
        double numer = m_region_agg->sample_total(m_clk_core_idx, region_hash);
        double denom = m_region_agg->sample_total(m_clk_ref_idx, region_hash);
        double freq = denom != 0 ? 100.0 * numer / denom : 0.0;
        report << "    frequency (%): " << freq << std::endl;
        report << "    frequency (Hz): " << freq / 100.0 * m_platform_io.read_signal("CPUINFO::FREQ_STICKER", GEOPM_DOMAIN_BOARD, 0) << std::endl;
        double network_time = application_io.total_region_runtime_mpi(region_hash);
        report << "    network-time (sec): " << network_time << std::endl;
        report << "    count: " << count << std::endl;
        for (const auto &env_it : m_env_signal_name_idx) {
            report << "    " << env_it.first << ": " << m_region_agg->sample_total(env_it.second, region_hash) << std::endl;
        }
        for (const auto &kv : agent_region_report) {
            report << "    " << kv.first << ": " << kv.second << std::endl;
        }
    }

    void ReporterImp::format_epoch(std::ostream &report,
                                   const ApplicationIO &application_io,
                                   const std::vector<std::pair<std::string, std::string> > &agent_region_report) const
    {
        report << "Epoch Totals:" << std::endl;
        double sync_rt = m_region_agg->sample_total(m_region_bulk_runtime_idx, GEOPM_REGION_HASH_EPOCH);
        double package_energy = m_region_agg->sample_total(m_energy_pkg_idx, GEOPM_REGION_HASH_EPOCH);
        double power = sync_rt == 0 ? 0 : package_energy / sync_rt;
        report << "    runtime (sec): " << application_io.total_epoch_runtime() << std::endl;
        report << "    sync-runtime (sec): " << sync_rt << std::endl;
        report << "    package-energy (joules): " << package_energy << std::endl;
        report << "    dram-energy (joules): " << m_region_agg->sample_total(m_energy_dram_idx, GEOPM_REGION_HASH_EPOCH) << std::endl;
        report << "    power (watts): " << power << std::endl;
        double numer = m_region_agg->sample_total(m_clk_core_idx, GEOPM_REGION_HASH_EPOCH);
        double denom = m_region_agg->sample_total(m_clk_ref_idx, GEOPM_REGION_HASH_EPOCH);
        double freq = denom != 0 ? 100.0 * numer / denom : 0.0;
        report << "    frequency (%): " << freq << std::endl;
        report << "    frequency (Hz): " << freq / 100.0 * m_platform_io.read_signal("CPUINFO::FREQ_STICKER", GEOPM_DOMAIN_BOARD, 0) << std::endl;
        double network_time = application_io.total_epoch_runtime_network();
        report << "    network-time (sec): " << network_time << std::endl;
        int count = application_io.total_epoch_count();
        report << "    count: " << count << std::endl;
        for (const auto &env_it : m_env_signal_name_idx) {
            report << "    " << env_it.first << ": " << m_region_agg->sample_total(env_it.second, GEOPM_REGION_HASH_EPOCH) << std::endl;
        }
        for (const auto &kv : agent_region_report) {
            report << "    " << kv.first << ": " << kv.second << std::endl;
        }
        report << "    epoch-runtime-ignore (sec): " << application_io.total_epoch_runtime_ignore() << std::endl;
    }

    void ReporterImp::format_app_totals(std::ostream &report,
                                        const ApplicationIO &application_io,
                                        const TreeComm &tree_comm) const
    {
        double total_runtime = m_platform_io.sample(m_app_time_signal_idx) - m_start_time_signal;
        double app_energy_pkg = m_platform_io.sample(m_app_energy_pkg_idx) - m_start_energy_pkg;
        double avg_power = total_runtime == 0 ? 0 : app_energy_pkg / total_runtime;
        double app_energy_dram = m_platform_io.sample(m_app_energy_dram_idx) - m_start_energy_dram;
        report << "Application Totals:" << std::endl
               << "    runtime (sec): " << total_runtime << std::endl
               << "    package-energy (joules): " << app_energy_pkg << std::endl
               << "    dram-energy (joules): " << app_energy_dram << std::endl
               << "    power (watts): " << avg_power << std::endl
               << "    network-time (sec): " << application_io.total_app_runtime_mpi() << std::endl
               << "    ignore-time (sec): " << application_io.total_app_runtime_ignore() << std::endl;

        std::string max_memory = get_max_memory();
        report << "    geopmctl memory HWM: " << max_memory << std::endl;
        report << "    geopmctl network BW (B/sec): " << tree_comm.overhead_send() / total_runtime << std::endl;
    }

    void ReporterImp::generate(const std::string &agent_name,
                               const std::vector<std::pair<std::string, std::string> > &agent_report_header,
                               const std::vector<std::pair<std::string, std::string> > &agent_host_report,
                               const std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > &agent_region_report,
                               const ApplicationIO &application_io,
                               std::shared_ptr<Comm> comm,
                               const TreeComm &tree_comm)
    {
        std::string report_name(application_io.report_name());
        if (report_name.size() == 0) {
            return;
        }

        /// todo: move rank check and aggregation out into controller
        int rank = comm->rank();
        std::ofstream master_report;

        if (!rank) {
            master_report.open(report_name);
            if (!master_report.good()) {
                throw Exception("Failed to open report file", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
            format_header(master_report, application_io.profile_name(), agent_name,
                          agent_report_header);
        }
        // per-host report
        std::ostringstream report;
        report << "\nHost: " << hostname() << std::endl;
        for (const auto &kv : agent_host_report) {
            report << kv.first << ": " << kv.second << std::endl;
        }

        format_all_regions(report, application_io, agent_region_report);

        std::vector<std::pair<std::string, std::string> > agent_epoch_ext;
        if (agent_region_report.find(GEOPM_REGION_HASH_EPOCH) != agent_region_report.end()) {
            agent_epoch_ext = agent_region_report.at(GEOPM_REGION_HASH_EPOCH);
        }
        format_epoch(report, application_io, agent_epoch_ext);

        format_app_totals(report, application_io, tree_comm);

        // aggregate reports from every node
        report.seekp(0, std::ios::end);
        size_t buffer_size = (size_t) report.tellp();
        report.seekp(0, std::ios::beg);
        std::vector<char> report_buffer;
        std::vector<size_t> buffer_size_array;
        std::vector<off_t> buffer_displacement;
        int num_ranks = comm->num_rank();
        buffer_size_array.resize(num_ranks);
        buffer_displacement.resize(num_ranks);
        comm->gather(&buffer_size, sizeof(size_t), buffer_size_array.data(),
                     sizeof(size_t), 0);

        if (!rank) {
            int full_report_size = std::accumulate(buffer_size_array.begin(), buffer_size_array.end(), 0) + 1;
            report_buffer.resize(full_report_size);
            buffer_displacement[0] = 0;
            for (int i = 1; i < num_ranks; ++i) {
                buffer_displacement[i] = buffer_displacement[i-1] + buffer_size_array[i-1];
            }
        }

        comm->gatherv((void *) (report.str().data()), sizeof(char) * buffer_size,
                      (void *) report_buffer.data(), buffer_size_array, buffer_displacement, 0);

        if (!rank) {
            report_buffer.back() = '\0';
            master_report << report_buffer.data();
            master_report << std::endl;
            master_report.close();
        }
    }

    std::string ReporterImp::get_max_memory()
    {
        char status_buffer[8192];
        status_buffer[8191] = '\0';
        const char *proc_path = "/proc/self/status";

        int fd = open(proc_path, O_RDONLY);
        if (fd == -1) {
            throw Exception("ReporterImp::generate(): Unable to open " + std::string(proc_path),
                            errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }

        ssize_t num_read = read(fd, status_buffer, 8191);
        if (num_read == -1) {
            (void)close(fd);
            throw Exception("ReporterImp::generate(): Unable to read " + std::string(proc_path),
                            errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        status_buffer[num_read] = '\0';

        int err = close(fd);
        if (err) {
            throw Exception("ReporterImp::generate(): Unable to close " + std::string(proc_path),
                            errno ? errno : GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }


        std::istringstream proc_stream(status_buffer);
        std::string line;
        std::string max_memory;
        const std::string key("VmHWM:");
        while (proc_stream.good()) {
            getline(proc_stream, line);
            if (line.find(key) == 0) {
                max_memory = line.substr(key.length());
                size_t off = max_memory.find_first_not_of(" \t");
                if (off != std::string::npos) {
                    max_memory = max_memory.substr(off);
                }
            }
        }
        if (!max_memory.size()) {
            throw Exception("Reporter::get_max_memory(): Unable to get memory overhead from /proc",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        return max_memory;
    }
}

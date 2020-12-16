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
#include "SampleAggregator.hpp"
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
                      SampleAggregator::make_unique(),
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
                             std::unique_ptr<SampleAggregator> agg,
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
    {
        m_sticker_freq = m_platform_io.read_signal("CPUINFO::FREQ_STICKER", GEOPM_DOMAIN_BOARD, 0);
    }

    void ReporterImp::init(void)
    {
        m_region_bulk_runtime_idx = m_region_agg->push_signal_total("TIME", GEOPM_DOMAIN_BOARD, 0);
        m_energy_pkg_idx = m_region_agg->push_signal_total("ENERGY_PACKAGE", GEOPM_DOMAIN_BOARD, 0);
        m_energy_dram_idx = m_region_agg->push_signal_total("ENERGY_DRAM", GEOPM_DOMAIN_BOARD, 0);
        m_clk_core_idx = m_region_agg->push_signal_total("CYCLES_THREAD", GEOPM_DOMAIN_BOARD, 0);
        m_clk_ref_idx = m_region_agg->push_signal_total("CYCLES_REFERENCE", GEOPM_DOMAIN_BOARD, 0);

        for (const std::string &signal_name : string_split(m_env_signals, ",")) {
            std::vector<std::string> signal_name_domain = string_split(signal_name, "@");
            if (signal_name_domain.size() == 2) {
                int domain_type = m_platform_topo.domain_name_to_type(signal_name_domain[1]);
                for (int domain_idx = 0; domain_idx < m_platform_topo.num_domain(domain_type); ++domain_idx) {
                    m_env_signal_name_idx.emplace_back(
                        signal_name + '-' + std::to_string(domain_idx),
                        m_region_agg->push_signal_total(signal_name_domain[0], domain_type, domain_idx));
                }
            }
            else if (signal_name_domain.size() == 1) {
                m_env_signal_name_idx.emplace_back(
                    signal_name,
                    m_region_agg->push_signal_total(signal_name, GEOPM_DOMAIN_BOARD, 0));
            }
            else {
                throw Exception("ReporterImp::init(): Environment report extension contains signals with multiple \"@\" characters.",
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
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
        m_region_agg->read_batch();
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

        int rank = comm->rank();
        std::ofstream common_report;
        if (!rank) {
            common_report.open(report_name);
            if (!common_report.good()) {
                throw Exception("Failed to open report file", GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
            // make header
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
            std::vector<std::pair<std::string, std::string> > header {
                {"GEOPM Version", geopm_version()},
                {"Start Time", m_start_time},
                {"Profile", application_io.profile_name()},
                {"Agent", agent_name},
                {"Policy", policy_str}
            };
            int indent_level = 0;
            yaml_write(common_report, indent_level, header);
            yaml_write(common_report, indent_level, agent_report_header);
        }

        // per-host report
        std::ostringstream report;
        report << "\n";
        yaml_write(report, 0, "Hosts:");
        yaml_write(report, 1, hostname() + ":");
        yaml_write(report, 2, agent_host_report);
        yaml_write(report, 2, "regions:");

        // vector of region data, in descending order by runtime
        std::vector<region_info> region_ordered;
        auto region_name_set = application_io.region_name_set();
        for (const auto &region : region_name_set) {
            uint64_t region_hash = geopm_crc32_str(region.c_str());
            int count = application_io.total_count(region_hash);
            if (count > 0) {
                region_ordered.push_back({region,
                                          region_hash,
                                          application_io.total_region_runtime(region_hash),
                                          application_io.total_region_runtime_mpi(region_hash),
                                          count});
            }
        }
        // sort based on averge runtime, descending
        std::sort(region_ordered.begin(), region_ordered.end(),
                  [] (const region_info &a,
                      const region_info &b) -> bool {
                      return a.per_rank_avg_runtime > b.per_rank_avg_runtime;
                  });

        for (const auto &region : region_ordered) {
#ifdef GEOPM_DEBUG
            if (GEOPM_REGION_HASH_INVALID == region.hash) {
                throw Exception("ReporterImp::generate(): Invalid hash value detected.",
                                GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
            }
#endif
            // TODO: get rid of numbers for indent level; probably need some vars
            yaml_write(report, 2, "-");
            yaml_write(report, 3, {{"name", region.name}});
            yaml_write(report, 3, {{"hash", region.hash}});
            auto region_data = get_region_data(region);
            yaml_write(report, 3, region_data);
            const auto &it = agent_region_report.find(region.hash);
            if (it != agent_region_report.end()) {
                yaml_write(report, 3, agent_region_report.at(region.hash));
            }
        }

        // TODO: once more of these ApplicationIO calls are removed, we can
        // get rid of the region_info struct

        yaml_write(report, 1, "Unmarked Totals:");
        double unmarked_time = application_io.total_region_runtime(GEOPM_REGION_HASH_UNMARKED);
        double unmarked_time_mpi = application_io.total_region_runtime_mpi(GEOPM_REGION_HASH_UNMARKED);
        region_info unmarked {"unmarked", GEOPM_REGION_HASH_UNMARKED,
            unmarked_time, unmarked_time_mpi, 0};
        auto unmarked_data = get_region_data(unmarked);
        yaml_write(report, 2, unmarked_data);
        // agent extensions for unmarked
        const auto &it = agent_region_report.find(GEOPM_REGION_HASH_UNMARKED);
        if (it != agent_region_report.end()) {
            yaml_write(report, 3, agent_region_report.at(GEOPM_REGION_HASH_UNMARKED));
        }


        yaml_write(report, 1, "Epoch Totals:");
        double epoch_runtime = application_io.total_epoch_runtime();
        double epoch_runtime_mpi = application_io.total_epoch_runtime_network();
        int epoch_count = application_io.total_epoch_count();
        region_info epoch {"epoch", GEOPM_REGION_HASH_EPOCH, epoch_runtime, epoch_runtime_mpi, epoch_count};
        auto epoch_data = get_region_data(epoch);
        // extra runtimes for epoch region
        epoch_data.push_back({"epoch-runtime-ignore (s)",
            application_io.total_epoch_runtime_ignore()});
        yaml_write(report, 2, epoch_data);

        yaml_write(report, 1, "Application Totals:");
        double total_runtime = m_region_agg->sample_total(m_region_bulk_runtime_idx,
                                                          GEOPM_REGION_HASH_INVALID);
        double total_runtime_mpi = application_io.total_app_runtime_mpi();
        // double app_energy_pkg = m_region_agg->sample_total(m_energy_pkg_idx);
        // double avg_power = total_runtime == 0 ? 0 : app_energy_pkg / total_runtime;
        // double app_energy_dram = m_region_agg->sample_total(m_energy_dram_idx);

        region_info app_totals {"totals", GEOPM_REGION_HASH_INVALID, total_runtime,
            total_runtime_mpi, 0};
        auto region_data = get_region_data(app_totals);
        yaml_write(report, 2, region_data);
        // TODO: app total ignore time is temporarily removed

        // // Note: this is "sync-runtime" total but there's very little
        // // difference with "accurate" runtime for app totals
        // double total_runtime = m_region_agg->sample_total(m_region_bulk_runtime_idx);
        // double app_energy_pkg = m_region_agg->sample_total(m_energy_pkg_idx);
        // double avg_power = total_runtime == 0 ? 0 : app_energy_pkg / total_runtime;
        // double app_energy_dram = m_region_agg->sample_total(m_energy_dram_idx);

        // report << "Application Totals:" << std::endl
        //        << "    runtime (sec): " << total_runtime << std::endl
        //        << "    package-energy (joules): " << app_energy_pkg << std::endl
        //        << "    dram-energy (joules): " << app_energy_dram << std::endl
        //        << "    power (watts): " << avg_power << std::endl
        //        << "    network-time (sec): " << application_io.total_app_runtime_mpi() << std::endl
        //        << "    ignore-time (sec): " << application_io.total_app_runtime_ignore() << std::endl;
        // for (const auto &env_it : m_env_signal_name_idx) {
        //     report << "    " << env_it.first << ": " << m_region_agg->sample_total(env_it.second) << std::endl;
        // }

        // TODO: can't mix string and double in these vectors.... annoying
        std::string max_memory = get_max_memory();
        yaml_write(report, 2, "geopmctl memory HWM: " + max_memory);

        std::vector<std::pair<std::string, double> > overhead {
            {"geopmctl network BW (B/s)", tree_comm.overhead_send() / total_runtime}
        };
        yaml_write(report, 2, overhead);

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
            common_report << report_buffer.data();
            common_report << std::endl;
            common_report.close();
        }
    }

    std::vector<std::pair<std::string, double> > ReporterImp::get_region_data(const region_info &region)
    {
        std::vector<std::pair<std::string, double> > result;
        double sync_rt = m_region_agg->sample_total(m_region_bulk_runtime_idx, region.hash);
        double package_energy = m_region_agg->sample_total(m_energy_pkg_idx, region.hash);
        double power = sync_rt == 0 ? 0 : package_energy / sync_rt;
        result.push_back({"runtime (s)", region.per_rank_avg_runtime});
        result.push_back({"sync-runtime (s)", sync_rt});
        result.push_back({"package-energy (J)", package_energy});
        result.push_back({"dram-energy (J)", m_region_agg->sample_total(m_energy_dram_idx, region.hash)});
        result.push_back({"power (W)", power});
        double numer = m_region_agg->sample_total(m_clk_core_idx, region.hash);
        double denom = m_region_agg->sample_total(m_clk_ref_idx, region.hash);
        double freq = denom != 0 ? 100.0 * numer / denom : 0.0;
        result.push_back({"frequency (%)", freq});
        result.push_back({"frequency (Hz)", freq / 100.0 * m_sticker_freq});
        result.push_back({"network-time (s)", region.network_time});
        result.push_back({"count", region.count});
        for (const auto &env_it : m_env_signal_name_idx) {
            result.push_back({env_it.first, m_region_agg->sample_total(env_it.second, region.hash)});
        }
        return result;
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
            throw Exception("Controller::generate_report(): Unable to get memory overhead from /proc",
                            GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        return max_memory;
    }

    void ReporterImp::yaml_write(std::ostream &os, int indent_level,
                                 const std::string &val)
    {
        std::string indent(indent_level * 2, ' ');
        os << indent << val << std::endl;
    }

    void ReporterImp::yaml_write(std::ostream &os, int indent_level,
                                 const std::vector<std::pair<std::string, std::string> > &data)
    {
        std::string indent(indent_level * 2, ' ');
        for (const auto &kv : data) {
            os << indent << kv.first << ": " << kv.second << std::endl;
        }
    }

    void ReporterImp::yaml_write(std::ostream &os, int indent_level,
                                 const std::vector<std::pair<std::string, double> > &data)
    {
        std::string indent(indent_level * 2, ' ');
        for (auto kv: data) {
            os << indent << kv.first << ": " << kv.second << std::endl;
        }
    }

}

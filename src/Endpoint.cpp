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

#include "Endpoint.hpp"

#include <cmath>
#include <string.h>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>

#include "contrib/json11/json11.hpp"

#include "Environment.hpp"
#include "PlatformTopo.hpp"
#include "SharedMemoryImp.hpp"
#include "Exception.hpp"
#include "Helper.hpp"
#include "Agent.hpp"
#include "config.h"

using json11::Json;

namespace geopm
{
    /// @todo: why is this not in SharedMemory?
    /// A: specific to endpoint shmem structure fields
    ///  could also have mutex in SharedMemory itself
    static void write_shmem(const std::unique_ptr<SharedMemory> &shmem,
                            const std::vector<double> &values)
    {
        struct geopm_endpoint_shmem_s *data_ptr = (struct geopm_endpoint_shmem_s *)shmem->pointer();
        int err = pthread_mutex_lock(&data_ptr->lock); // Default mutex will block until this completes.
        if (err) {
            throw Exception("ShmemEndpoint::pthread_mutex_lock()", err, __FILE__, __LINE__);
        }
        data_ptr->count = values.size();
        // @todo: agent and timestamp
        std::copy(values.begin(), values.end(), data_ptr->values);

        pthread_mutex_unlock(&data_ptr->lock);
    }

    static void read_shmem(const std::unique_ptr<SharedMemoryUser> &shmem,
                           std::vector<double> &values)
    {
        struct geopm_endpoint_shmem_s *data_ptr = (struct geopm_endpoint_shmem_s *)shmem->pointer(); // Managed by shmem subsystem.

        int err = pthread_mutex_lock(&data_ptr->lock); // Default mutex will block until this completes.
        if (err) {
            throw Exception("ShmemEndpointUser::pthread_mutex_lock()", err, __FILE__, __LINE__);
        }

        // Fill in missing policy values with NAN (default)
        std::fill(values.begin(), values.end(), NAN);
        //values = std::vector<double>(m_signal_names.size(), NAN);
        std::copy(data_ptr->values, data_ptr->values + data_ptr->count, values.begin());

        (void) pthread_mutex_unlock(&data_ptr->lock);
    }

    // todo: move me
    const std::string SHM_POLICY_POSTFIX = "-policy";
    const std::string SHM_SAMPLE_POSTFIX = "-sample";

    ShmemEndpoint::ShmemEndpoint(const std::string &data_path, const std::string &agent_name)
        : ShmemEndpoint(data_path,
                        nullptr,
                        nullptr)
    {
    }

    ShmemEndpoint::ShmemEndpoint(const std::string &path,
                                 std::unique_ptr<SharedMemory> policy_shmem,
                                 std::unique_ptr<SharedMemory> sample_shmem)
        : m_path(path)
        , m_policy_shmem(std::move(policy_shmem))
        , m_sample_shmem(std::move(sample_shmem))
        , m_samples_up(0)
    {
        if (m_policy_shmem == nullptr) {
            size_t shmem_size = sizeof(struct geopm_endpoint_shmem_s);
            m_policy_shmem = geopm::make_unique<SharedMemoryImp>(m_path + SHM_POLICY_POSTFIX, shmem_size);
        }
        if (m_sample_shmem == nullptr) {
            size_t shmem_size = sizeof(struct geopm_endpoint_shmem_s);
            m_sample_shmem = geopm::make_unique<SharedMemoryImp>(m_path + SHM_SAMPLE_POSTFIX, shmem_size);
        }

        auto data = (struct geopm_endpoint_shmem_s *) m_shmem->pointer();
        *data = {};
        setup_mutex(data->lock);
    }

    FileEndpoint::FileEndpoint(const std::string &data_path)
        : FileEndpoint(data_path, environment().agent())
    {
    }

    FileEndpoint::FileEndpoint(const std::string &data_path, const std::string &agent_name)
        : FileEndpoint(data_path,
                       Agent::policy_names(agent_factory().dictionary(agent_name)))
    {

    }

    FileEndpoint::FileEndpoint(const std::string &path,
                               const std::vector<std::string> &policy_names)
        : m_path(path)
        , m_policy_names(policy_names)
        , m_samples_up(signal_names.size())
    {

    }

    void ShmemEndpoint::setup_mutex(pthread_mutex_t &lock)
    {
        pthread_mutexattr_t lock_attr;
        int err = pthread_mutexattr_init(&lock_attr);
        if (err) {
            throw Exception("ProfileTable: pthread mutex initialization", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        err = pthread_mutexattr_settype(&lock_attr, PTHREAD_MUTEX_ERRORCHECK);
        if (err) {
            throw Exception("ProfileTable: pthread mutex initialization", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        err = pthread_mutexattr_setpshared(&lock_attr, PTHREAD_PROCESS_SHARED);
        if (err) {
            throw Exception("ProfileTable: pthread mutex initialization", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
        err = pthread_mutex_init(&lock, &lock_attr);
        if (err) {
            throw Exception("ProfileTable: pthread mutex initialization", GEOPM_ERROR_RUNTIME, __FILE__, __LINE__);
        }
    }

    void ShmemEndpoint::write_policy(const std::vector<double> &policy)
    {
        if (policy.size() != m_signal_names.size()) {
            throw Exception("ShmemEndpoint::" + std::string(__func__) + "(): size of policy does not match signal names.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        m_samples_up = policy;
        write_shmem(m_policy_shmem, policy);
    }

    void FileEndpoint::write_policy(const std::vector<double> &policy)
    {
        if (policy.size() != m_policy_names.size()) {
            throw Exception("FileEndpoint::" + std::string(__func__) + "(): size of policy does not match signal names.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        write_file(policy);
    }

    // std::vector<std::string> ShmemEndpoint::signal_names(void) const
    // {
    //     return m_signal_names;
    // }

    // std::vector<std::string> FileEndpoint::signal_names(void) const
    // {
    //     return m_signal_names;
    // }

    void FileEndpoint::write_file(const std::vector<double> &values)
    {
        std::map<std::string, double> signal_value_map;
        for(size_t i = 0; i < m_policy_names.size(); ++i) {
            signal_value_map[m_policy_names[i]] = values[i];
        }

        Json root(signal_value_map);
        geopm::write_file(m_path, root.dump());
    }

    // void ShmemEndpoint::write_shmem(void)
    // {
    //     geopm::write_shmem(m_shmem, m_samples_up);
    // }

    int ShmemEndpoint::read_sample(std::vector<double> &sample)
    {
        return -1;
    }

    std::string ShmemEndpoint::get_agent(void)
    {
        throw Exception("ShmemEndpoint::" + std::string(__func__) + "(): get_agent not yet supported",
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
        return "";
    }

    int FileEndpoint::read_sample(std::vector<double> &sample)
    {
        throw Exception("FileEndpoint::" + std::string(__func__) + "(): sending samples via file not yet supported",
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
        return -1;
    }

    std::string FileEndpoint::get_agent(void)
    {
        throw Exception("FileEndpoint::" + std::string(__func__) + "(): get_agent via file not yet supported",
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
        return "";
    }

    /*********************************************************************************************************/

    ShmemEndpointUser::ShmemEndpointUser(const std::string &data_path, const std::string &agent_name)
        : ShmemEndpointUser(data_path,
                            nullptr,
                            is_policy ? Agent::policy_names(agent_factory().dictionary(agent_name)) :
                                        Agent::sample_names(agent_factory().dictionary(agent_name)))
    {
    }

    ShmemEndpointUser::ShmemEndpointUser(const std::string &path,
                                         std::unique_ptr<SharedMemoryUser> policy_shmem,
                                         std::unique_ptr<SharedMemoryUser> sample_shmem)
        : m_path(path)
        , m_policy_shmem(std::move(policy_shmem))
        , m_sample_shmem(std::move(sample_shmem))
    {
        /// @todo: need to read_policy() here?
        std::vector<double> temp(signal_names.size());
        read_policy(temp);



    }

    FileEndpointUser::FileEndpointUser(const std::string &data_path, const std::string &agent_name)
        : FileEndpointUser(data_path,
                           Agent::policy_names(agent_factory().dictionary(agent_name)))
    {
    }

    FileEndpointUser::FileEndpointUser(const std::string &path,
                                       const std::vector<std::string> &policy_names)
        : m_path(path)
        , m_policy_names(policy_names)
    {
        /// @todo: need to read_policy() here?
        std::vector<double> temp(m_policy_names.size());
        read_policy(temp);
    }

    std::map<std::string, double> FileEndpointUser::parse_json(void)
    {
        std::map<std::string, double> signal_value_map;
        std::string json_str;

        json_str = read_file(m_path);

        // Begin JSON parse
        std::string err;
        Json root = Json::parse(json_str, err);
        if (!err.empty() || !root.is_object()) {
            throw Exception("FileEndpointUser::" + std::string(__func__) + "(): detected a malformed json config file: " + err,
                            GEOPM_ERROR_FILE_PARSE, __FILE__, __LINE__);
        }

        for (const auto &obj : root.object_items()) {
            if (obj.second.type() == Json::NUMBER) {
                signal_value_map.emplace(obj.first, obj.second.number_value());
            }
            else if (obj.second.type() == Json::STRING) {
                std::string tmp_val = obj.second.string_value();
                if (tmp_val.compare("NAN") == 0 || tmp_val.compare("NaN") == 0 || tmp_val.compare("nan") == 0) {
                    signal_value_map.emplace(obj.first, NAN);
                }
                else {
                    throw Exception("FileEndpointUser::" + std::string(__func__)  + ": unsupported type or malformed json config file",
                                    GEOPM_ERROR_FILE_PARSE, __FILE__, __LINE__);
                }
            }
            else {
                throw Exception("FileEndpointUser::" + std::string(__func__)  + ": unsupported type or malformed json config file",
                                GEOPM_ERROR_FILE_PARSE, __FILE__, __LINE__);
            }
        }

        return signal_value_map;
    }

    void ShmemEndpointUser::read_shmem(void)
    {
        if (m_policy_shmem == nullptr) {
            m_policy_shmem = geopm::make_unique<SharedMemoryUserImp>(m_path + SHM_POLICY_POSTFIX,
                                                                     environment().timeout());
            //@todo: don't throw for timeout; instead return a timestamp indicating no policy ready yet
        }

        m_signals_down.resize(m_signal_names.size());

        geopm::read_shmem(m_shmem, m_signals_down);

        if (m_signals_down.size() != m_signal_names.size()) {
            throw Exception("ShmemEndpointUser::" + std::string(__func__) + "(): Data read from shmem does not match size of signal names.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
    }

    int ShmemEndpointUser::read_policy(std::vector<double> &policy)
    {
        read_shmem();
        policy = m_signals_down;
        return -1; // @todo -1 indicates not ready
    }

    int FileEndpointUser::read_policy(std::vector<double> &policy)
    {
        if (m_policy_names.size() > 0) {
            std::map<std::string, double> signal_value_map = parse_json();
            m_signals_down.clear();
            for (auto signal : m_policy_names) {
                auto it = signal_value_map.find(signal);
                if (it != signal_value_map.end()) {
                    m_signals_down.emplace_back(signal_value_map.at(signal));
                }
                else {
                    // Fill in missing policy values with NAN (default)
                    m_signals_down.emplace_back(NAN);
                }
            }
        }
        policy = m_signals_down;
        return -1;  //@todo
    }

    void ShmemEndpointUser::write_sample(const std::vector<double> &sample)
    {
        // if (m_sample_shmem == nullptr) {
        //     m_sample_shmem = geopm::make_unique<SharedMemoryUserImp>(m_path + SHM_SAMPLE_POSTFIX,
        //                                                              environment().timeout());
        // }
        /// @todo: timeout is not an error; just throw out the sample

    }

    void FileEndpointUser::write_sample(const std::vector<double> &sample)
    {
        throw Exception("FileEndpoint::" + std::string(__func__) + "(): sending samples via file not yet supported",
                        GEOPM_ERROR_NOT_IMPLEMENTED, __FILE__, __LINE__);
    }
}

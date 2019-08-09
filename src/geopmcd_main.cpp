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

#include <unistd.h>
#include <getopt.h>

#include <iostream>
#include <fstream>

#include "geopm_time.h"
#include "Endpoint.hpp"
#include "Agent.hpp"
#include "Helper.hpp"
#include "PolicyStore.hpp"


// TODO list
// - Change to endpoint C api
// - one-off query commands to view policy, latest sample, attached agent, profile, nodelist
// - command to apply a static policy
// - create/destroy endpoint?
// - version and help
// - configuration for daemon

template <typename T>
std::ostream& operator<<(std::ostream& os, std::vector<T> vec)
{
    os << "{";
    for (int ii = 0; ii < vec.size(); ++ii) {
        os << vec[ii];
        if (ii < vec.size() - 1) {
            os << ", ";
        }
    }
    os << "}";
    return os;
}

std::ostream& operator<<(std::ostream& os, geopm_time_s time)
{
    os << (double)(time.t.tv_sec + time.t.tv_nsec * 1.0E-9);
    return os;
}

int main(int argc, char **argv)
{
    /// Handle command line arguments
    static struct option long_options[] = {
        {"daemon", no_argument, NULL, 'd'},
        {"help", no_argument, NULL, 'h'},
        {"version", no_argument, NULL, 'v'},
        {NULL, 0, NULL, 0}
    };

    int err = 0;
    int opt;
    bool is_daemon = false;
    bool do_help = false;
    bool do_version = false;
    while (!err && (opt = getopt_long(argc, argv, "dhv", long_options, NULL)) != -1) {
        arg_ptr = NULL;
        switch (opt) {
            case 'd':
                is_daemon = true;
                break;
            case 'h':
                do_help = true;
                break;
            case 'v':
                do_version = true;
                break;
            case '?': // opt is ? when an option required an arg but it was missing
                do_help = true;
                err = EINVAL;
                break;
            default:
                fprintf(stderr, "Error: getopt returned character code \"0%o\"\n", opt);
                err = EINVAL;
                break;
        }
    }

    if (do_help) {
        // print usage string
    }
    if (do_version) {
        printf("%s\n", geopm_version());
        printf("\n\nCopyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation. All rights reserved.\n\n");
    }
    if (do_help || do_version) {
        return err;
    }

    /// Switch between command mode and daemon mode.  Applying static
    /// policies in command mode will use the PolicyStore; for this to
    /// work, the policy path must be set in the environment override,
    /// and the SQLite policy store DB must be created for the user.
    /// In daemon mode, the power_governor agent must be used and the
    /// policy will be a power cap based on the time of day.



    if (is_daemon) {
        std::string shmem_prefix = "/geopmcd_endpoint_test";
        auto endpoint = geopm::Endpoint::make_unique(shmem_prefix);
        while (true) {
            endpoint->open();

            // check for agent
            std::string agent = "";
            std::string profile_name;
            while (agent == "") {
                agent = endpoint->get_agent();
            }
            std::cout << "Controller with agent " << agent << " attached." << std::endl;
            if (agent != "power_governor") {
                std::cout << "power_governor will use dynamic policy." << std::endl;
            }
            else {
                std::cout << agent << " will use policy from policy store." << std::endl;
                auto policy_store = geopm::PolicyStore::make_unique("/home/drguttma/policystore.db");
                profile_name = endpoint->get_profile_name();
                policy_store->get_best(profile_name, agent);
            }
            std::ofstream log_file("endpoint_test.log");
            std::vector<double> sample(geopm::Agent::num_sample(geopm::agent_factory().dictionary(agent)));

            int offset = 0;
            geopm_time_s start_time;
            geopm_time(&start_time);
            geopm_time_s last_sample_time = start_time;
            while (agent != "") {
                endpoint.write_policy({11.0 + offset});
                offset = (int)geopm_time_since(&start_time) % 60;

                // get sample or timeout
                double TIMEOUT = 3.0;
                geopm_time_s sample_time;
                geopm_time_s current_time;
                geopm_time_s zero {{0, 0}};
                do {
                    geopm_time(&current_time);
                    sample_time = endpoint->read_sample(sample);
                }
                while (geopm_time_diff(&sample_time, &zero) == 0.0 ||
                       (geopm_time_diff(&sample_time, &last_sample_time) == 0.0 &&
                        geopm_time_diff(&last_sample_time, &current_time) < TIMEOUT));

                if (geopm_time_diff(&last_sample_time, &current_time) >= TIMEOUT) {
                    std::cerr << "Timeout waiting for Controller sample." << std::endl;
                    agent = "";
                    endpoint->close();
                }
                else {
                    last_sample_time = sample_time;
                    log_file << sample_time << " " << sample << std::endl;
                    agent = endpoint->get_agent();
                }
            }
            std::cout << "Controller detached." << std::endl;
        }

        // TODO: handle SIGINT so that endpoint can cleanup
        endpoint->close();
    }
}

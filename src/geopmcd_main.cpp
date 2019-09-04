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
#include <signal.h>

#include <iostream>
#include <fstream>

#include "geopm_time.h"
#include "geopm_version.h"
#include "Endpoint.hpp"
#include "Agent.hpp"
#include "Helper.hpp"
#include "PolicyStore.hpp"


// TODO list
// - Change to endpoint C api
// - one-off query commands to view policy, latest sample, attached agent, profile, nodelist
// - command to apply a static policy
// - create/destroy endpoint?
// - configuration for daemon
// - allow overwriting file policy in command mode
// - break out functions

/// Helpers for printing
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

static bool g_continue = true;
static void handler(int sig)
{
    g_continue = false;
}


int main(int argc, char **argv)
{
    /// Handle signals
    struct sigaction act;
    act.sa_handler = handler;
    sigaction(SIGINT, &act, NULL);

    /// Handle command line arguments
    const char *usage = "\n"
        "Usage: geopmendpoint\n";
    static struct option long_options[] = {
        {"daemon", no_argument, NULL, 'd'},
        {"set-policy", required_argument, NULL, 'p'},
        {"view-policy", no_argument, NULL, 'P'},
        {"view-sample", no_argument, NULL, 'S'},
        {"view-agent", no_argument, NULL, 'A'},
        {"help", no_argument, NULL, 'h'},
        {"version", no_argument, NULL, 'v'},
        {NULL, 0, NULL, 0}
    };

    int err = 0;
    int opt;
    char *arg_ptr = NULL;
    bool is_daemon = false;
    bool default_policy = true;
    bool do_help = false;
    bool do_version = false;
    bool view_agent = false;
    while (!err && (opt = getopt_long(argc, argv, "dp:PSAhv", long_options, NULL)) != -1) {
        arg_ptr = NULL;
        switch (opt) {
            case 'd':
                is_daemon = true;
                break;
            case 'A':
                view_agent = true;
                default_policy = false;
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
        printf("%s", usage);
    }
    if (do_version) {
        printf("%s\n", geopm_version());
        printf("\n\nCopyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation. All rights reserved.\n\n");
    }
    if (do_help || do_version) {
        return err;
    }
    if (is_daemon && (view_agent)) {
        std::cerr << "Warning: daemon mode is incompatible with other options." << std::endl;
    }

    /// Switch between command mode and daemon mode.  Applying static
    /// policies in command mode will use the PolicyStore; for this to
    /// work, the policy path must be set in the environment override,
    /// and the SQLite policy store DB must be created for the user.
    /// In daemon mode, the power_governor agent must be used and the
    /// policy will be a power cap based on the time of day.
    if (is_daemon) {
        std::string shmem_prefix = "/geopmcd_endpoint_test";
        //auto endpoint = geopm::Endpoint::make_unique(shmem_prefix);
        auto endpoint = std::make_shared<geopm::ShmemEndpoint>(shmem_prefix);
        while (g_continue) {
            endpoint->open();

            // check for agent
            std::string agent = "";
            std::string profile_name;
            while (agent == "") {
                agent = endpoint->get_agent();
            }
            std::cout << "Controller with agent " << agent << " attached." << std::endl;
            if (is_daemon && agent == "power_governor") {
                std::cout << "power_governor will use dynamic policy." << std::endl;
            }
            // todo: move this out
            else if (!is_daemon) {
                std::cout << agent << " will use policy from policy store." << std::endl;
                auto policy_store = geopm::PolicyStore::make_unique("/home/drguttma/policystore.db");
                profile_name = endpoint->get_profile_name();
                auto policy = policy_store->get_best(profile_name, agent);
                // todo: apply policy and exit
                std::cout << "Got policy: " << policy << std::endl;
                endpoint->write_policy(policy);
                endpoint->close();
                return 0;
            }
            else {
                std::cerr << "Warning: daemon mode not supported for agents other than power_governor.  "
                          << "No policy will be applied." << std::endl;
                /// @todo: just warn and use policystore?

            }

            // todo: check daemon mode here to avoid early return above
            std::ofstream log_file("endpoint_test.log");
            std::vector<double> sample(geopm::Agent::num_sample(geopm::agent_factory().dictionary(agent)));

            /// @todo: should be board total, not per package
            int num_pkg = geopm_topo_num_domain(GEOPM_DOMAIN_PACKAGE);
            double board_tdp = 0.0;
            err = geopm_pio_read_signal("POWER_PACKAGE_TDP", GEOPM_DOMAIN_BOARD, 0, &result);
            board_tdp *= num_pkg;
            double range = 30;  // how much to vary power caps over time

            void write_dynamic_power_policy(std::shared_ptr<Endpoint> endpoint,
                                            struct geopm_time_s start_time)
            {
                static int offset = 0;

                static bool once = true;
                if (once) {
                    geopm_time(&start_time);
                    once = false;
                }
                endpoint->write_policy({board_tdp - range + offset});
                offset = (int)geopm_time_since(&start_time) % range;
            }

            void get_sample_or_timeout(struct geopm_time_s *last_sample_time,
                                       std::string *agent)
            {
                const geopm_time_s zero {{0, 0}};
                const double TIMEOUT = 3.0;
                geopm_time_s sample_time;
                geopm_time_s current_time;
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
                    *last_sample_time = sample_time;
                    log_file << sample_time << " " << sample << std::endl;
                    *agent = endpoint->get_agent();
                }
            }


            struct geopm_time_s last_sample_time;
            geopm_time(&last_sample_time);

            while (g_continue && agent != "") {
                static geopm_time_s start_time;
                geopm_time(&start_time);

                write_dynamic_power_policy(endpoint, start_time);

                get_sample_or_timeout(&last_sample_time, &agent);

            }
            std::cout << "Controller detached." << std::endl;
        } // endwhile g_continue
        endpoint->close();
    } // endif is_daemon
    else {
        if (view_agent) {
            // Display agent, profile, node list
            auto agent = endpoint->
        }

    }
    return 0;
}

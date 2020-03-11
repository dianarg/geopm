/*
 * Copyright (c) 2016, 2017, 2018, 2019, 2020, Intel Corporation
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

// Steps to demo:
//
// 1. populate policystore file with
//    tutorial/admin/initialize_policystore.py (path is hardcoded
//    below)
//
// 2. update /etc/geopm/environment-*.json with GEOPM_ENDPOINT
//    pointing to endpoint (hardcoded below)
//
// 3. build and run this binary on the compute node.  You may need
//    to manually delete shared memory to start.
//    > rm -f /dev/shm/geopm* && geopmcd
//
// 4. allocate the configured node and run with geopmlaunch with no
//    profile.  The policy should be the default for the agent.
//    > geopmlaunch srun -N1 -n1 --geopm-trace-endpoint-policy=policy.trace -- geopmbench
//    > tail policy.trace
//
//
// 5. run with the profile given in initialize_policystore.py
//    (myprofile).  The policy should match what was setup as the best
//    policy.
//    > geopmlaunch srun -N1 -n1 --geopm-trace-endpoint-policy=policy.trace --geopm-profile=myprofile -- geopmbench
//    > tail policy.trace


#include "config.h"

#include <csignal>

#include <string>
#include <iostream>

#include "Daemon.hpp"
#include "OptionParser.hpp"

volatile bool g_continue_flag;
std::unique_ptr<geopm::Daemon> g_daemon;

void handle_signal(int signum)
{
    g_continue_flag = false;
    if (g_daemon) {
        try {
            g_daemon->stop_wait_loop();
        }
        catch (geopm::Exception &ex) {
            std::cerr << "geopmcd: stop_wait_loop() failed while handling signal." << std::endl;
        }
    }
}

int main(int argc, char *argv[])
{
    g_continue_flag = true;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // todo: try catch around c++
    // todo: want log files instead of stdout?
    geopm::OptionParser parser {"geopmcd", std::cout, std::cerr};
    parser.add_option("status", 's', "status", false, "get current status of geopmcd service");
    bool early_exit = parser.parse(argc, argv);
    if (early_exit) {
        return 0;
    }
    if (parser.is_set("status")) {
        std::cout << "STATUS" << std::endl;
    }
    else {
        std::cout << "STARTING GEOPM SERVICE" << std::endl;
        std::string endpoint_name = "geopmcd_test_endpoint";
        // TODO: put in /etc/geopm?
        std::string db_path = "geopmcd_policystore.db";
        std::cout << "endpoint=\"" << endpoint_name << "\"" << std::endl;
        std::cout << "policystore=\"" << db_path << "\"" << std::endl;
        try {
            g_daemon = geopm::Daemon::make_unique(endpoint_name, db_path);
        }
        catch (geopm::Exception &ex) {
            std::cerr << "geopmcd: Failed to start daemon: " << ex.what() << std::endl;
        }

        // Handle a single agent at a time, sending it a policy from the policystore
        while (g_continue_flag) {
            std::cout << "geopmcd: waiting for controller..." << std::endl;
            try {
                g_daemon->update_endpoint_from_policystore(-1);
                if (g_continue_flag) {
                    std::cout << "geopmcd: completed update from policystore." << std::endl;
                }
            }
            catch (geopm::Exception &ex) {
                std::cout << "geopmcd: failed to assign policy: " << ex.what() << std::endl;
            }
            catch (std::exception &ex) {
                std::cerr << "geopmcd: some other error: " << ex.what() << std::endl;
            }

            // not needed unless we call stop_wait_loop()
            // TODO: might want to handle a signal to reset?
            //g_daemon->reset_wait_loop();
        }
        std::cout << "STOPPING GEOPM SERVICE" << std::endl;
    }

    return 0;
}

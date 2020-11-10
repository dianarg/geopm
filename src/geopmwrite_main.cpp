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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <cmath>

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <vector>

#include "geopm_version.h"
#include "geopm_error.h"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Exception.hpp"
#include "OptionParser.hpp"

#include "config.h"

using geopm::PlatformIO;
using geopm::PlatformTopo;
using geopm::OptionParser;

int parse_domain_type(const std::string &dom);

int main(int argc, char **argv)
{
    OptionParser parser("geopmwrite", std::cout, std::cerr);
    parser.add_option("domain", 'd', "domain", false, "print domains detected");
    parser.add_option("info", 'i', "info", false, "print longer description of a control");
    parser.add_option("info_all", 'I', "info-all", false, "print longer description of all controls");
    parser.add_option("cache", 'c', "cache", false, "create geopm topo cache if it does not exist");
    parser.add_example_usage("CONTROL_NAME DOMAIN_TYPE DOMAIN_INDEX VALUE");
    parser.add_example_usage("[--info [CONTROL_NAME]]");
    parser.add_example_usage("[--info-all]");
    parser.add_example_usage("[--domain]");
    parser.add_example_usage("[--cache]");
    parser.add_usage_description("  CONTROL_NAME:  name of the control\n"
                                 "  DOMAIN_TYPE:   name of the domain for which the control should be written\n"
                                 "  DOMAIN_INDEX:  index of the domain, starting from 0\n"
                                 "  VALUE:         setting to adjust control to");
    bool do_exit = parser.parse(argc, argv);
    if (do_exit) {
        return 0;
    }
    if (parser.is_set("cache")) {
        geopm::PlatformTopo::create_cache();
        return 0;
    }

    int err = 0;
    bool is_domain = parser.is_set("domain");
    bool is_info = parser.is_set("info");
    bool is_all_info = parser.is_set("info_all");

    if (is_domain && is_info) {
        std::cerr << "Error: info about domain not implemented." << std::endl;
        return EINVAL;
    }

    std::vector<std::string> pos_args;
    while (optind < argc) {
        pos_args.emplace_back(argv[optind++]);
    }

    PlatformIO &platform_io = geopm::platform_io();
    const PlatformTopo &platform_topo = geopm::platform_topo();
    if (is_domain) {
        // print all domains
        for (int dom = GEOPM_DOMAIN_BOARD; dom < GEOPM_NUM_DOMAIN; ++dom) {
            std::cout << std::setw(24) << std::left
                      << PlatformTopo::domain_type_to_name(dom)
                      << platform_topo.num_domain(dom) << std::endl;
        }
    }
    else if (is_info) {
        try {
            // print description for one control
            if (pos_args.size() > 0) {
                std::cout << pos_args[0] << ":" << std::endl
                          << platform_io.control_description(pos_args[0]) << std::endl;
            }
            else {
                std::cerr << "Error: no control requested." << std::endl;
                err = EINVAL;
            }
        }
        catch (const geopm::Exception &ex) {
            std::cerr << "Error: " << ex.what() << std::endl;
            err = EINVAL;
        }
    }
    else if (is_all_info) {
        // print all controls with description
        auto controls = platform_io.control_names();
        for (const auto &con : controls) {
            std::cout << con << ":" << std::endl
                      << platform_io.control_description(con) << std::endl;
        }
    }
    else {
        if (pos_args.size() == 0) {
            // print all controls
            auto controls = platform_io.control_names();
            for (const auto &con : controls) {
                std::cout << con << std::endl;
            }
        }
        else if (pos_args.size() >= 4) {
            // write control
            std::string control_name = pos_args[0];
            int domain_idx = -1;
            double write_value = NAN;
            try {
                domain_idx = std::stoi(pos_args[2]);
            }
            catch (const std::invalid_argument&) {
                std::cerr << "Error: invalid domain index.\n" << std::endl;
                err = EINVAL;
            }
            if (!err) {
                try {
                    write_value = std::stod(pos_args[3]);
                }
                catch (const std::invalid_argument&) {
                    std::cerr << "Error: invalid write value.\n" << std::endl;
                    err = EINVAL;
                }
            }
            if (!err) {
                try {
                    int domain_type = PlatformTopo::domain_name_to_type(pos_args[1]);
                    int idx = platform_io.push_control(control_name, domain_type, domain_idx);
                    platform_io.adjust(idx, write_value);
                    platform_io.write_batch();
                }
                catch (const geopm::Exception &ex) {
                    std::cerr << "Error: cannot write control: " << ex.what() << std::endl;
                    err = EINVAL;
                }
            }
        }
        else {
            std::cerr << "Error: domain type, domain index, and value are required to write control.\n" << std::endl;
            err = EINVAL;
        }
    }

    return err;
}

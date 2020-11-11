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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <string>
#include <stdexcept>
#include <iostream>
#include <iomanip>

#include "geopm_version.h"
#include "geopm_error.h"
#include "geopm_hash.h"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Exception.hpp"
#include "OptionParser.hpp"
#include "Helper.hpp"


using geopm::PlatformIO;
using geopm::PlatformTopo;
using geopm::OptionParser;

int parse_domain_type(const std::string &dom);

int main(int argc, char **argv)
{
    OptionParser parser("geopmread", std::cout, std::cerr);
    parser.add_option("domain", 'd', "domain", false, "print domains detected");
    parser.add_option("info", 'i', "info", false, "print longer description of a signal");
    parser.add_option("info_all", 'I', "info-all", false, "print longer description of all signals");
    parser.add_option("cache", 'c', "cache", false, "create geopm topo cache if it does not exist");
    parser.add_option("label", 'l', "label", false, "label output values with signal details");
    parser.add_option("batch", 'b', "batch", false, "use read_batch interface instead of read_signal");

    parser.add_example_usage("[--batch] [--label] SIGNAL_NAME0,DOMAIN_TYPE0,DOMAIN_INDEX0 [NAME1,TYPE1,INDEX1 ...]");
    parser.add_example_usage("[--info [SIGNAL_NAME]]");
    parser.add_example_usage("[--info-all]");
    parser.add_example_usage("[--domain]");
    parser.add_example_usage("[--cache]");
    parser.add_usage_description("  SIGNAL_NAME:  name of the signal\n"
                                 "  DOMAIN_TYPE:  name of the domain for which the signal should be read\n"
                                 "  DOMAIN_INDEX: index of the domain, starting from 0"
                                 );
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
    bool add_label = parser.is_set("label");
    bool use_batch = parser.is_set("batch");

    if (is_domain && is_info) {
        std::cerr << "Error: info about domain not implemented." << std::endl;
        return EINVAL;
    }

    std::vector<std::string> pos_args = parser.get_positional_args();

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
            // print description for one signal
            if (pos_args.size() > 0) {
                std::cout << pos_args[0] << ":" << std::endl
                          << platform_io.signal_description(pos_args[0]) << std::endl;
            }
            else {
                std::cerr << "Error: no signal requested." << std::endl;
                err = EINVAL;
            }
        }
        catch (const geopm::Exception &ex) {
            std::cerr << "Error: " << ex.what() << std::endl;
            err = EINVAL;
        }
    }
    else if (is_all_info) {
        // print all signals with description
        auto signals = platform_io.signal_names();
        for (const auto &sig : signals) {
            std::cout << sig << ":" << std::endl
                      << platform_io.signal_description(sig) << std::endl;
        }
    }
    else {
        if (pos_args.size() == 0) {
            // print all signals
            auto signals = platform_io.signal_names();
            for (const auto &sig : signals) {
                std::cout << sig << std::endl;
            }
        }
        else {
            // read signals
            std::vector<int> signal_idx;
            std::vector<std::string> signal_names;
            for (auto const& bundle : pos_args) {
                auto pieces = geopm::string_split(bundle, ",");
                if (pieces.size() != 3) {
                    std::cerr << "Error: signal request must be a comma-separated tuple of name, domain type, domain index." << std::endl;
                    return EINVAL;
                }
                std::string signal_name = pieces[0];
                int domain_type = -1;
                int domain_idx = -1;
                try {
                    domain_type = PlatformTopo::domain_name_to_type(pieces[1]);
                }
                catch (const geopm::Exception &ex) {
                    std::cerr << "Error: invalid domain index: " << pieces[1] << std::endl;
                    err = EINVAL;
                }
                try {
                    domain_idx = std::stoi(pieces[2]);
                }
                catch (const std::invalid_argument &) {
                    std::cerr << "Error: invalid domain index: " << pieces[2] << std::endl;
                    err = EINVAL;
                }
                if (use_batch) {
                    try {
                        int idx = platform_io.push_signal(signal_name, domain_type, domain_idx);
                        signal_idx.push_back(idx);
                        signal_names.push_back(signal_name);
                    }
                    catch (const geopm::Exception &ex) {
                        std::cerr << "Error: cannot push signal: " << ex.what() << std::endl;
                        err = EINVAL;
                    }
                }
                else {
                    try {
                        double result = platform_io.read_signal(signal_name, domain_type, domain_idx);
                        std::string label = "";
                        if (add_label) {
                            label = signal_name + ": ";
                        }
                        std::cout << label << platform_io.format_function(signal_name)(result) << std::endl;
                    }
                    catch (const geopm::Exception &ex) {
                        std::cerr << "Error: cannot read signal: " << ex.what() << std::endl;
                        err = EINVAL;
                    }
                }
            }
            if (use_batch) {
                if (signal_idx.size() != signal_names.size()) {
                    std::cerr << "Something went wrong with size of arrays" << std::endl;
                    return EINVAL;
                }
                try {
                    platform_io.read_batch();
                }
                catch (const geopm::Exception &ex) {
                    std::cerr << "Error: cannot read batch: " << ex.what() << std::endl;
                    err = EINVAL;
                }
                for (int ii = 0; ii < signal_idx.size(); ++ii) {
                    try {
                        double result = platform_io.sample(signal_idx.at(ii));
                        std::string name = signal_names.at(ii);
                        std::string label = "";
                        if (add_label) {
                            label = name + ": ";
                        }
                        std::cout << label << platform_io.format_function(name)(result) << std::endl;
                    }
                    catch (const geopm::Exception &ex) {
                        std::cerr << "Error: cannot sample signal: " << ex.what() << std::endl;
                        err = EINVAL;
                    }

                }
            }
        }
    }
    return err;
}

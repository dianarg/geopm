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

#include <mpi.h>
#include <cmath>
#include <string>
#include <vector>
#include <memory>

#include "geopm.h"
#include "Profile.hpp"
#include "Exception.hpp"
#include "ModelRegion.hpp"

int main(int argc, char **argv)
{
    // Start MPI
    int comm_rank;
    MPI_Init(&argc, &argv);
    MPI_Comm_rank(MPI_COMM_WORLD, &comm_rank);
    // Parse command line option for verbosity
    bool is_verbose = false;
    if (comm_rank == 0) {
        for (int arg_idx = 1; arg_idx < argc; ++arg_idx) {
            std::string arg(argv[arg_idx]);
            if (arg == "--verbose" || arg == "-v") {
                is_verbose = true;
            }
        }
    }

    geopm::Profile &prof = geopm::Profile::default_profile();
    uint64_t rid0 = prof.region("MyRegion", GEOPM_REGION_HINT_COMPUTE);
    // TODO: change to a different hint
    uint64_t rid1 = prof.region("MyRegion_nested", GEOPM_REGION_HINT_NETWORK);
    // setup time
    uint64_t rid2 = prof.region("Setup", GEOPM_REGION_HINT_IGNORE);
    prof.enter(rid2);

    std::unique_ptr<geopm::ModelRegion> user_region_outer(
        geopm::ModelRegion::model_region("spin", 0.5, is_verbose));
    std::unique_ptr<geopm::ModelRegion> user_region_inner(
        geopm::ModelRegion::model_region("spin", 0.25, is_verbose));
    std::unique_ptr<geopm::ModelRegion> mpi_region(
        geopm::ModelRegion::model_region("all2all-unmarked", 1.0, is_verbose));
    std::unique_ptr<geopm::ModelRegion> omp_region(
        geopm::ModelRegion::model_region("scaling-unmarked", 1.0, is_verbose));

    prof.exit(rid2);

    int num_step = 10;
    for (int idx = 0; idx != num_step; ++idx) {
        prof.epoch();
        prof.enter(rid0);
        user_region_outer->run();
        prof.enter(rid1);
        user_region_inner->run();
        MPI_Barrier(MPI_COMM_WORLD);
        prof.exit(rid1);
        prof.exit(rid0);

        // bare OpenMP region
        omp_region->run();

        // bare MPI region
        mpi_region->run();

    }
    // Shutdown MPI
    MPI_Finalize();
    return 0;
}

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

#include <string.h>
#include <mpi.h>
#include <string>
#include <vector>

#include "geopm.h"
#include "ModelRegion.hpp"


int main(int argc, char **argv)
{
    int err = 0;
    int comm_rank;
    int comm_size;
    MPI_Init(&argc, &argv);
    MPI_Comm_size(MPI_COMM_WORLD, &comm_size);
    MPI_Comm_rank(MPI_COMM_WORLD, &comm_rank);
    bool is_verbose = false;
    if (comm_rank == 0) {
        for (int arg_idx = 1; arg_idx < argc; ++arg_idx) {
            if (strncmp(argv[arg_idx], "--verbose", strlen("--verbose")) == 0 ||
                strncmp(argv[arg_idx], "-v", strlen("-v")) == 0) {
                is_verbose = true;
            }
        }
    }
    int repeat = 500;
    std::vector<std::string> regions {"stream-unmarked", "dgemm-unmarked"};
    geopm::ModelRegion *sleep_model = geopm::model_region_factory("sleep", 0.1, is_verbose);
    double dgemm_factor = 6.0
    double stream_factor = 0.5
    int num_mix = 11;
    double max_factor = 1.0 / (num_mix - 1);
    std::vector<std::pair<double, double> > big_o_list;
    for (int mix_idx = 0;  !err && mix_idx != num_mix; ++mix_idx) {
        stream_idx = num_mix - 1 - mix_idx;
        dgemm_idx = mix_idx;
        stream_big_o = stream_factor * mix_factor * stream_idx;
        dgemm_big_o = dgemm_factor * mix_factor * dgemm_idx;
        std::string stream_name("stream-" + std::to_string(stream_idx));
        std::string dgemm_name("dgemm-" + std::to_string(dgemm_idx));
        ModelRegionBase *stream_model = model_region_factory(stream_name, stream_big_o, is_verbose);
        ModelRegionBase *dgemm_model = model_region_factory(dgemm_name, dgemm_big_o, is_verbose);
        std::string region_name(stream_name + '-' + dgemm_name);
        uint64_t region_id;
        err = geopm_prof_region(region_name.c_str(), GEOPM_REGION_HINT_UNKNOWN, &region_id);
        if (err) goto exit;
        for (int rep_idx = 0; rep_idx != repeat; ++rep_idx) {
            err = geopm_prof_enter(region_id);
            if (err) goto exit;
            stream_model->run();
            dgemm_model->run();
            err = geopm_prof_exit(region_id);
            if (err) goto exit;
            MPI_Comm_barrier(MPI_COMM_WORLD);
        }
exit:
        delete dgemm_model;
        delete stream_model;
    }
    MPI_Finalize();
    return err;
}

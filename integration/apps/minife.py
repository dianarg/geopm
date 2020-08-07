#  Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in
#        the documentation and/or other materials provided with the
#        distribution.
#
#      * Neither the name of Intel Corporation nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

import os

from apps import apps


class MinifeAppConf(apps.AppConf):
    @staticmethod
    def name():
        return 'minife'

    @staticmethod
    def is_geopm_linked():
        return True

    def __init__(self, num_nodes):
        self.ranks_per_node = 2
        problem_sizes = {
            1: '-nx=396 -ny=384 -nz=384',  # '-nx=264 -ny=256 -nz=256',
            4: '-nx=528 -ny=512 -nz=512',  # '-nx=419 -ny=406 -nz=406',
            8: '-nx=792 -ny=768 -nz=768',
            64: '-nx=1056 -ny=1024 -nz=1024',
            128: '-nx=1584 -ny=1536 -nz=1536',   # '-nx=1330 -ny=1290 -nz=1290',
            256: '-nx=1676 -ny=1625 -nz=1625',
            512: '-nx=2112 -ny=2048 -nz=2048',  # scale each dimension of 1-node size by 512^(1/3)=8
        }
        if num_nodes not in problem_sizes:
            raise RuntimeError("No input size defined for minife on {} nodes".format(self.num_nodes))
        self.app_params = problem_sizes[num_nodes]
        # TODO: find a better place for this config
        benchmark_dir = '$HOME/benchmarks/geopm_markup'
        self.exe_path = os.path.join(benchmark_dir, 'minife/miniFE_openmp-2.0-rc3/src/miniFE.x')

    def get_rank_per_node(self):
        return self.ranks_per_node

    def setup(self):
        return ''

    def get_exec_path(self):
        return self.exe_path

    def get_exec_args(self):
        return self.app_params

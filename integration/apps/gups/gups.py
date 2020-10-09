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

'''
Describes the best known configuration for GUPS, the RandomAccess
benchmark from HPC Challenge.
'''

import os

from .. import apps


class GupsAppConf(apps.AppConf):

    @staticmethod
    def name():
        return 'gups'

    def __init__(self, add_barriers=False):
        self._add_barriers = add_barriers
        benchmark_dir = os.path.dirname(os.path.abspath(__file__))
        self._exec_path = os.path.join(benchmark_dir, 'gups/ra_mpiomp')
        self._num_rank_per_node = 2
        self._exec_args = ['20']  # table size, 2^N; should be <= 1/2 main memory

    def get_rank_per_node(self):
        # TODO: use machine_file to determine
        return self._num_rank_per_node

    def get_cpu_per_rank(self):
        # Number of threads must be a power of 2
        return 16

    def get_bash_exec_path(self):
        return self._exec_path

    def get_bash_exec_args(self):
        return self._exec_args

    def parse_fom(self, log_path):
        result = None
        key = 'Billion(10^9) Updates    per second [GUP/s]'
        # Note: this is shared (appended) by all jobs.  Choose the
        # last one and hope there's no race
        out_path = 'hpccoutf.txt'
        with open(out_path) as fid:
            for line in fid.readlines():
                if key in line:
                    result = float(line.split()[0])
                    # don't break, overwrite with last occurence
        return result

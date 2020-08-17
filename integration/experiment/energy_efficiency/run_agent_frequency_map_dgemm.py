#!/usr/bin/env python
#
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
Run DGEMM with the frequency map agent.
'''

import argparse

from experiment import common_args
from apps import geopmbench
from experiment import machine
from experiment.energy_efficiency import energy_agent


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    common_args.add_output_dir(parser)
    common_args.add_nodes(parser)
    common_args.add_reference_frequency(parser)
    common_args.add_policy_file(parser)

    args, experiment_cli_args = parser.parse_known_args()

    output_dir = args.output_dir
    num_node = args.nodes
    policy_file = args.policy_file

    # reference run will create a different policy, so this argument
    # should not be passed
    # TODO: doesn't work with = syntax
    if '--geopm-policy' in experiment_cli_args or '--geopm-endpoint' in experiment_cli_args:
        raise RuntimeError('Use --policy-file to pass path to policy; do not use \
                            --geopm-policy or --geopm-endpoint.')

    if policy_file is None or policy_file == '':
        raise RuntimeError('A valid frequency_map agent policy file must be provided using \
                            --policy-file.  Run gen_frequency_map.py \
                            on the results of a frequency_sweep experiment.')

    mach = machine.init_output_dir(output_dir)
    ref_freq = args.reference_frequency
    if not ref_freq:
        ref_freq = mach.frequency_sticker()

    # application parameters
    app_conf = geopmbench.DgemmAppConf(output_dir)

    # experiment parameters
    iterations = 2

    energy_agent.launch_frequency_map_agent(output_dir=output_dir,
                                            iterations=iterations,
                                            num_node=num_node,
                                            app_conf=app_conf,
                                            policy_file=policy_file,
                                            reference_freq=ref_freq,
                                            experiment_cli_args=experiment_cli_args)

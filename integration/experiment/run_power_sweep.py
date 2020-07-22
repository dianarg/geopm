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
Example power sweep experiment using geopmbench.
'''

import sys
import os
import math
import pandas
import glob

import geopmpy.io

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from integration.experiment.power_sweep import launch_power_sweep
from integration.util import sys_power_avail
from integration.test.util import do_launch


def setup_power_bounds(min_power, max_power, step_power):
    sys_min, sys_tdp, sys_max = sys_power_avail()
    if min_power is None or min_power < sys_min:
        # system minimum is actually too low; use 50% of TDP or min rounded up to nearest step, whichever is larger
        min_power = max(int(0.5 * sys_tdp), sys_min)
        min_power = int(step_power * math.ceil(float(min_power)/step_power))
        sys.stderr.write("Warning: <geopm> run_power_sweep: Invalid or unspecified min_power; using default minimum: {}.\n".format(min_power))
    if max_power is None or max_power > sys_max:
        max_power = sys_tdp
        sys.stderr.write("Warning: <geopm> run_power_sweep: Invalid or unspecified max_power; using system TDP: {}.\n".format(max_power))
    return min_power, max_power


# TODO: utility function?  only needs the name
def find_report_files(search_pattern='*report'):
    """
    Uses the output dir and any custom name prefix to discover reports
    produced by launch.
    """
    # TODO: add output dir
    reports = glob.glob(search_pattern)
    reports = sorted(reports)
    return reports


def summary(parse_output):
    # rename some columns
    parse_output['power_limit'] = parse_output['POWER_PACKAGE_LIMIT_TOTAL']
    parse_output['runtime'] = parse_output['runtime (sec)']
    parse_output['network_time'] = parse_output['network-time (sec)']
    parse_output['energy_pkg'] = parse_output['package-energy (joules)']
    parse_output['energy_dram'] = parse_output['dram-energy (joules)']
    parse_output['frequency'] = parse_output['frequency (Hz)']
    parse_output['iteration'] = parse_output.apply(lambda row: row['Profile'].split('_')[-1],
                                                   axis=1)
    # set up index for grouping
    parse_output = parse_output.set_index(['Agent', 'host', 'power_limit'])
    summary = pandas.DataFrame()
    for col in ['count', 'runtime', 'network_time', 'energy_pkg', 'energy_dram', 'frequency']:
        summary[col] = parse_output[col].groupby(['Agent', 'power_limit']).mean()
    return summary


if __name__ == '__main__':
    # TODO: can dynamically choose with setup_power_bounds(), command line options
    min_power = 180
    max_power = 200
    step_power = 10
    name = 'bench'

    # TODO: handle num node and rank correctly

    do_launch = do_launch()
    if do_launch:
        application = "geopmbench"
        launch_power_sweep(file_prefix=name,
                           output_dir='.',
                           detailed=True,
                           iterations=2,
                           min_power=min_power,
                           max_power=max_power,
                           step_power=step_power,
                           agent_types=['power_governor', 'power_balancer'],
                           num_node=1,
                           num_rank=1,
                           launcher_name='srun',
                           args=['-n1', '-N1', application])

    # TODO: must match output_dir, currently '.'
    reports = find_report_files(name + '*report')
    print(reports)
    output = geopmpy.io.RawReportCollection(reports)

    result = summary(output.get_epoch_df())

    sys.stdout.write('{}\n'.format(result))

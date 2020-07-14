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

import geopmpy.io

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from integration.experiment.power_sweep import PowerSweep
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


def find_report_files(self, search_pattern='*report'):
    """
    Uses the output dir and any custom naming convention to load the report
    produced by launch.
    """
    report_glob = os.path.join(self._output_dir, self._name + search_pattern)
    report_files = [os.path.basename(ff) for ff in glob.glob(report_glob)]
    reports = []
    for report in report_files:
        try:
            power = int(report.split('_')[1])
            reports.append(report)
        except:
            pass
    return reports


if __name__ == '__main__':
    # TODO: can dynamically choose with setup_power_bounds(), command line options
    min_power = 180
    max_power = 200
    step_power = 10

    # TODO: AppConf?  might want to move to common util
    application = "geopmbench"

    sweep = PowerSweep(profile_prefix='bench',
                       output_dir='.',
                       detailed=True,
                       iterations=1,   # todo: move to launch?
                       min_power=min_power,
                       max_power=max_power,
                       step_power=step_power,
                       agent_type='power_governor')

    # TODO: handle num node and rank

    do_launch = do_launch()
    if do_launch:
        sweep.launch(num_node=1,
                     num_rank=1,
                     launcher_name='srun',
                     args=['-n1', '-N1', application])

    reports = sweep.find_report_files()
    output = geopmpy.io.RawReportCollection(reports)

    sweep.summary(output.get_epoch_df)

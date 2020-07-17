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

# TODO: some methods from test/util.py could move here

# Helper functions useful for both tests and experiments

import sys
import os

import geopmpy.launcher

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from integration.test import geopm_test_launcher


def sys_power_avail():
    # TODO: might want a common compute node launcher outside of test
    min_power = geopm_test_launcher.geopmread("POWER_PACKAGE_MIN board 0")
    tdp_power = geopm_test_launcher.geopmread("POWER_PACKAGE_TDP board 0")
    max_power = geopm_test_launcher.geopmread("POWER_PACKAGE_MAX board 0")
    return min_power, tdp_power, max_power


def try_launch(launcher_name, app_argv, report_path, trace_path, profile_name, agent_conf):
    if app_argv:
        argv = ['dummy', '--geopm-report', report_path,
                         '--geopm-trace', trace_path,
                         '--geopm-profile', profile_name]
        if agent_conf.get_agent() != 'monitor':
            argv.append('--geopm-agent=' + agent_conf.get_agent())
            argv.append('--geopm-policy=' + agent_conf.get_path())
        argv.extend(app_argv)
        argv.insert(1, launcher_name)
        launcher = geopmpy.launcher.Factory().create(argv)
        launcher.run()
    else:
        raise RuntimeError('<geopm> util.try_launch(): no application was specified.\n'.format(report_path))

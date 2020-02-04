#!/usr/bin/env python
#
#  Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
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

from __future__ import absolute_import


import os
import sys
import unittest
import shlex
import subprocess
import io
import json

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_integration import geopm_context
import geopmpy.pio
import geopmpy.topo
from test_integration import util
import geopmpy.io


def getSystemConfig():
    try:
        subprocess.check_call(shlex.split("geopmadmin"),
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError:
        sys.stdout.write("geopmadmin check failed, there is an issue with the site configuration.\n")
        pass
    settings = {}
    for option in ["--config-default", "--config-override"]:
        try:
            proc = subprocess.Popen(shlex.split("geopmadmin {}".format(option)),
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            config_file = proc.stdout.readline()
            proc = subprocess.Popen(shlex.split("cat {}".format(config_file)),
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            settings.update(json.loads(proc.communicate()[0]))
        except ValueError:
            # config_file may not exist and will cause Popen to fail
            pass
    return settings


def getSystemConfigAgent():
    ret = ''
    try:
        ret = getSystemConfig()['GEOPM_AGENT']
    except LookupError:
        pass
    return ret


def getSystemConfigPolicy():
    geopm_system_config = getSystemConfig()
    proc = subprocess.Popen(shlex.split("cat {}".format(geopm_system_config['GEOPM_POLICY'])),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return json.loads(proc.communicate()[0])


@util.skip_unless_batch()
class TestIntegrationPluginStaticPolicy(unittest.TestCase):
    """Test the static policy enforcement feature of the currently
       configured RM plugin.

    """
    @classmethod
    def setUpClass(cls):
        cls._tmp_files = []
        cls._geopmadminagent = getSystemConfigAgent()
        cls._geopmadminagentpolicy = getSystemConfigPolicy()

    @unittest.skipUnless(getSystemConfigAgent() in ['frequency_map', 'energy_efficient'],
                         'Requires environment default/override to be configured to cap frequency.')
    def test_frequency_cap_enforced(self):
        if self._geopmadminagent == 'frequency_map':
            test_freq = self._geopmadminagentpolicy['FREQ_MAX']
        elif self._geopmadminagent == 'energy_efficient':
            test_freq = self._geopmadminagentpolicy['FREQ_FIXED']
        current_freq = geopmpy.pio.read_signal("MSR::PERF_CTL:FREQ", "board", 0)
        self.assertEqual(test_freq, current_freq)

    @unittest.skipUnless(getSystemConfigAgent() in ['power_governor', 'power_balancer'],
                         'Requires environment default/override to be configured to cap power.')
    def test_power_cap_enforced(self):
        num_pkg = geopmpy.topo.num_domain('package')
        test_power = self._geopmadminagentpolicy['POWER_PACKAGE_LIMIT_TOTAL'] / num_pkg
        for pkg in range(num_pkg):
            current_power = geopmpy.pio.read_signal("MSR::PKG_POWER_LIMIT:PL1_POWER_LIMIT", "package", pkg)
            self.assertEqual(test_power, current_power)

    # TODO: move shared code to util.py
    def assert_geopm_uses_agent_policy(self, expected_agent, expected_policy, context, user_policy=None):
        """Assert that geopm uses the given policy.
        Arguments:
        expected_policy (dict str->float): Policy to expect in generated reports.
        context (str): Additional context for test file names and failure messages.
        user_policy (dict str->float): Policy to request in the geopmlaunch command.
        """
        report_path = '{}.report'.format(context)
        num_node = 1
        num_rank = 1
        app_conf = geopmpy.io.BenchConf(context + '.app.config')
        self._tmp_files.append(app_conf.get_path())
        app_conf.append_region('sleep', 0.01)
        user_agent_conf = geopmpy.io.AgentConf(context + '.user.agent.config', 'monitor', {} if user_policy is None else user_policy)
        self._tmp_files.append(user_agent_conf.get_path())
        launcher = geopm_test_launcher.TestLauncher(app_conf, user_agent_conf, report_path)
        launcher.set_num_node(num_node)
        launcher.set_num_rank(num_rank)
        launcher.run(context, include_geopm_policy=user_policy is not None)
        self._tmp_files.append(report_path)

        actual_agent = geopmpy.io.RawReport(report_path).meta_data()['Agent']
        self.assertEqual(expected_agent, actual_agent)
        actual_policy = geopmpy.io.RawReport(report_path).meta_data()['Policy']
        for expected_key, expected_value in expected_policy.items():
            self.assertEqual(expected_value, actual_policy[expected_key],
                             msg='Wrong policy value for {} (context: {})'.format(expected_key, context))
        for actual_key, actual_value in actual_policy.items():
            if actual_key not in expected_policy:
                self.assertEqual('NAN', actual_value,
                                 msg='Unexpected value for {} (context: {})'.format(actual_key, context))

    @unittest.skipUnless(getSystemConfigAgent() != '',
                         'Requires environment default/override files to be configured with an agent')
    def test_agent_enforced(self):
        test_name = 'test_agent_enforced'
        config = getSystemConfig()
        agent = config['GEOPM_AGENT']
        policy = config['GEOPM_POLICY']
        self.assert_geopm_uses_agent_policy(expected_agent=agent, expected_policy=policy, context=test_name)


if __name__ == '__main__':
    unittest.main()

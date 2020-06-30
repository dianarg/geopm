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

"""POWER_SWEEP

"""

import sys
import unittest
import os
import glob
import math
import pandas

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_integration import geopm_context
import geopmpy.io
import geopmpy.error

from test_integration import util
if util.do_launch():
    # Note: this import may be moved outside of do_launch if needed to run
    # commands on compute nodes such as geopm_test_launcher.geopmread
    from test_integration import geopm_test_launcher
    geopmpy.error.exc_clear()

class AppConf(object):
    """Class that is used by the test launcher in place of a
    geopmpy.io.BenchConf when running the power_sweep benchmark.

    """
    def write(self):
        """Called by the test launcher prior to executing the test application
        to write any files required by the application.

        """
        pass

    def get_exec_path(self):
        """Path to benchmark filled in by template automatically.

        """
        script_dir = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(script_dir, '.libs', 'test_power_sweep')

    def get_exec_args(self):
        """Returns a list of strings representing the command line arguments
        to pass to the test-application for the next run.  This is
        especially useful for tests that execute the test-application
        multiple times.

        """
        return []


class TestIntegration_power_sweep(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Create launcher, execute benchmark and set up class variables.

        """
        sys.stdout.write('(' + os.path.basename(__file__).split('.')[0] +
                         '.' + cls.__name__ + ') ...')



        test_name = 'power_sweep'
        cls._prefix = 'test_{}_'.format(test_name)
        cls._skip_launch = not util.do_launch()
        cls._keep_files = (cls._skip_launch or
                           os.getenv('GEOPM_KEEP_FILES') is not None)
        cls._agent_conf_path = 'test_' + test_name + '-agent-config.json'
        # Clear out exception record for python 2 support
        geopmpy.error.exc_clear()

        cls._target_report_dir = '/home/drguttma/output/last_dgemm'
        return
        if not cls._skip_launch:
            # Set the job size parameters
            num_node = 1
            num_rank = 4
            time_limit = 6000
            # Configure the test application
            app_conf = AppConf()
            # Configure the agent
            sys_min = geopm_test_launcher.geopmread("POWER_PACKAGE_MIN board 0")
            sys_tdp = geopm_test_launcher.geopmread("POWER_PACKAGE_TDP board 0")
            #sys_max = geopm_test_launcher.geopmread("POWER_PACKAGE_MAX board 0")
            cls._step_power = 10

            # system minimum is actually too low; use 50% of TDP or min rounded up to nearest step, whichever is larger
            cls._min_power = max(int(0.5 * sys_tdp), sys_min)
            cls._min_power = int(cls._step_power * math.ceil(float(cls._min_power)/cls._step_power))
            cls._max_power = int(sys_tdp)

            for power_cap in range(cls._min_power, cls._max_power+1, cls._step_power):
                agent_conf_dict = {'POWER_PACKAGE_LIMIT_TOTAL': power_cap}
                agent_conf = geopmpy.io.AgentConf(cls._agent_conf_path,
                                                  'power_governor',
                                                  agent_conf_dict)
                # Create the test launcher with the above configuration
                launcher = geopm_test_launcher.TestLauncher(app_conf,
                                                            agent_conf,
                                                            cls._prefix + str(power_cap) + '.report',
                                                            cls._prefix + str(power_cap) + '.trace',
                                                            time_limit=time_limit)
                launcher.set_num_node(num_node)
                launcher.set_num_rank(num_rank)
                # Run the test application
                launcher.run('test_' + test_name)

    @classmethod
    def tearDownClass(cls):
        """Clean up any files that may have been created during the test if we
        are not handling an exception and the GEOPM_KEEP_FILES
        environment variable is unset.

        """
        pass
        # if not cls._keep_files:
        #     os.unlink(cls._agent_conf_path)
        #     os.unlink(cls._report_path)
        #     for tf in glob.glob(cls._trace_path + '.*'):
        #         os.unlink(tf)

    def tearDown(self):
        if sys.exc_info() != (None, None, None):
            TestIntegration_power_sweep._keep_files = True

    def test_summary(self):
        #self._output_dir = '.'
        #output = geopmpy.io.AppOutput(self._prefix+'*.report', None, dir_name=self._output_dir, verbose=True)
        #output.extract_index_from_profile(inplace=True)

        report_glob = self._prefix+'*.report'
        if cls._target_report_dir:
            report_glob = cls._target_report_dir + '/' + report_glob

        cc = geopmpy.io.RawReportCollection(report_glob)
        df = cc.get_epoch_df()
        sys.stdout.write('{}\n'.format(df))
        return

        # profile name has been changed to power cap
        df = output.get_report_data(profile=(self._min_power, self._max_power),
                                    agent=self._agent_type,
                                    region='epoch')
        summary = pandas.DataFrame()
        for col in ['count', 'runtime', 'network_time', 'energy_pkg', 'energy_dram', 'frequency']:
            summary[col] = df[col].groupby(level='name').mean()
        summary.index.rename('power cap', inplace=True)

        rs = 'Summary for power sweep'
        #rs = 'Summary for {} with {} agent\n'.format(self._name, self._agent_type)
        rs += summary.to_string()
        sys.stdout.write(rs + '\n')
        report = geopmpy.io.RawReport(self._report_path)

    def test_achieved_freq_histogram(self):
        pass

    def test_runtime_energy_plot(self):
        def generate_runtime_energy_plot(df, name, output_dir='.'):
            """
            Creates a plot comparing the runtime and energy of a region on two axes.
            """
            f, ax = plt.subplots()

            ax.plot(df.index, df['energy_pkg'], color='DarkBlue', label='Energy', marker='o', linestyle='-')
            ax.set_xlabel('Power (W)')
            ax.set_ylabel('Energy (J)')

            ax2 = ax.twinx()
            ax2.plot(df.index, df['runtime'], color='Red', label='Runtime', marker='s', linestyle='-')
            ax2.set_ylabel('Runtime (s)')

            lines, labels = ax.get_legend_handles_labels()
            lines2, labels2 = ax2.get_legend_handles_labels()

            plt.legend(lines + lines2, labels + labels2, shadow=True, fancybox=True, loc='best')
            plt.title('{}'.format(name))
            f.tight_layout()
            plt.savefig(os.path.join(output_dir, '{}_runtime_energy.{}'.format(name.replace(' ', '_'), 'png')))
            plt.close()

        report_glob = '*power_governor*.report'
        cc = geopmpy.io.RawReportCollection(report_glob)
        report_df = cc.get_epoch_df()
        print(report_df)
        return

        regions = report_df.index.get_level_values('region').unique().tolist()
        process_output = {region: self._runtime_energy_sweep(parse_output, region)
                for region in regions}
        for region, df in process_output.items():
            geopmpy.plotter.generate_runtime_energy_plot(df, region, self._output_dir)


        #report_glob = str(power_cap) + '*.report'
        #cc = geopmpy.io.RawReportCollection(report_glob)

        for power_cap in (280):
            df = cc.get_epoch_df()
            generate_runtime_energy_plot(df, 'Runtime/Energy vs. Power')


if __name__ == '__main__':
    # Call do_launch to clear non-pyunit command line option
    util.do_launch()
    unittest.main()

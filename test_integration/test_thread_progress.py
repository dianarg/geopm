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

"""THREAD_PROGRESS

"""

import sys
import unittest
import os
import glob

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
    geopmpy.io.BenchConf when running the thread_progress benchmark.

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
        return os.path.join(script_dir, '.libs', 'test_thread_progress')

    def get_exec_args(self):
        """Returns a list of strings representing the command line arguments
        to pass to the test-application for the next run.  This is
        especially useful for tests that execute the test-application
        multiple times.

        """
        return []


class TestIntegration_thread_progress(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Create launcher, execute benchmark and set up class variables.

        """
        sys.stdout.write('(' + os.path.basename(__file__).split('.')[0] +
                         '.' + cls.__name__ + ') ...')
        test_name = 'thread_progress'
        cls._report_path = 'test_{}.report'.format(test_name)
        cls._trace_path = 'test_{}.trace'.format(test_name)
        cls._skip_launch = not util.do_launch()
        cls._keep_files = (cls._skip_launch or
                           os.getenv('GEOPM_KEEP_FILES') is not None)
        cls._agent_conf_path = test_name + '-agent-config.json'
        # Clear out exception record for python 2 support
        geopmpy.error.exc_clear()
        if not cls._skip_launch:
            # Set the job size parameters
            num_node = 1
            num_rank = 1
            time_limit = 6000
            # Configure the test application
            app_conf = AppConf()
            # Configure the agent
            agent_conf = geopmpy.io.AgentConf(cls._agent_conf_path, 'monitor')
            trace_signals = "REGION_THREAD_PROGRESS"
            # Create the test launcher with the above configuration
            launcher = geopm_test_launcher.TestLauncher(app_conf,
                                                        agent_conf,
                                                        cls._report_path,
                                                        cls._trace_path,
                                                        time_limit=time_limit,
                                                        trace_signals=trace_signals)
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

        if not cls._keep_files:
            os.unlink(cls._agent_conf_path)
            os.unlink(cls._report_path)
            for tf in glob.glob(cls._trace_path + '.*'):
                os.unlink(tf)

    def tearDown(self):
        if sys.exc_info() != (None, None, None):
            TestIntegration_thread_progress._keep_files = True

    # @unittest.expectedFailure  # TODO Remove when feature is fixed
    def test_load_report(self):
        """Test that the report can be loaded

        """
        output = geopmpy.io.AppOutput(self._report_path, self._trace_path + '*')
        for nn in output.get_node_names():
            trace = output.get_trace_data(node_name=nn)
            # todo get separate df for rows in unmarked region and named region
            progress = trace['REGION_THREAD_PROGRESS']
            sys.stdout.write('{}\n'.format(progress[:10]))
            sys.stdout.write('{}\n'.format(progress[-10:]))
            first = progress[0]
            last = progress[-1]
            self.assertNotEqual(first, last, "thread progress should change over time")


if __name__ == '__main__':
    # Call do_launch to clear non-pyunit command line option
    util.do_launch()
    unittest.main()

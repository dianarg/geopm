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

"""File containing all tests that can be executed based on using the
monitor agent and a single sleep region.

"""

import sys

if '--bench' in sys.argv:
    # When --bench is given the script is an MPI application that
    # executes the sleep region.

    import geopmpy.bench
    import geopmpy.prof
    import ctypes
    # Load the GEOPM PMPI wrappers prior to importing mpi4py
    ctypes.CDLL('libgeopm.so', mode=ctypes.RTLD_GLOBAL)
    import mpi4py.MPI

    def bench_main():
        """Function that is executed when the script is run as a benchmark.

        """
        is_verbose = (mpi4py.MPI.COMM_WORLD.Get_rank() == 0 and
                      ('--verbose' in sys.argv or '-v' in sys.argv))
        model_region = geopmpy.bench.model_region_factory('sleep', 1.0, is_verbose)
        geopmpy.bench.model_region_run(model_region)
        geopmpy.prof.shutdown()

    if __name__ == '__main__':
        bench_main()

else:
    import unittest
    import os
    import geopm_context
    import geopmpy.io
    import geopm_test_launcher

    class AppConf(object):
        """Class that is used by the test launcher as a geopmpy.io.BenchConf
        when running the script as a benchmark.

        """
        def write(self):
            """No configuration files are required.

            """
            pass

        def get_exec_path(self):
            """Execute this script.

            """
            result = os.path.realpath(__file__)
            if result.endswith('.pyc'):
                result = result[:-1]
            return result

        def get_exec_args(self):
            """Pass this script --bench to indicate that MPI application should be
            run.

            """
            return ['--bench']


    class TestIntegrationMonitorSleep(unittest.TestCase):
        @classmethod
        def setUpClass(cls):
            """Create launcher, execute this script as a benchmark and store the
            output.

            """
            test_name = 'test_monitor_sleep'
            cls._num_node = 4
            cls._num_rank = 16
            app_conf = AppConf()
            agent_conf = geopmpy.io.AgentConf(test_name + '-agent-config.json')
            cls._report_path = test_name + '.report'
            cls._launcher = geopm_test_launcher.TestLauncher(app_conf, agent_conf, cls._report_path)
            cls._launcher.set_num_node(cls._num_node)
            cls._launcher.set_num_rank(cls._num_rank)
            cls._launcher.run(test_name)
            cls._output = geopmpy.io.AppOutput(cls._report_path)

        @classmethod
        def tearDownClass(cls):
            """If we are not handling an exception and the GEOPM_KEEP_FILES
            environment variable is unset, clean up output.

            """

            if (sys.exc_info() == (None, None, None) and
                os.getenv('GEOPM_KEEP_FILES') is None):
                cls._output.remove_files()
                cls._launcher.remove_files()

        def test_report_exists(self):
            """Test that a report is generated."""
            self.assertTrue(os.path.exists(self._report_path))

        def test_number_nodes(self):
            """Test that report has the correct number of nodes.

            """
            node_names = self._output.get_node_names()
            self.assertEqual(self._num_node, len(node_names))

        def test_report_not_empty(self):
            """Test that reports are not empty.

            """
            node_names = self._output.get_node_names()
            for nn in node_names:
                report = self._output.get_report_data(node_name=nn)
                self.assertNotEqual(0, len(report))

    if __name__ == '__main__':
        unittest.main()

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

import sys
_do_bench = False
if '--bench' in sys.argv:
    _do_bench = True
    import geopmpy.bench
    import geopmpy.prof
    import ctypes
    ctypes.CDLL('libgeopm.so', mode=ctypes.RTLD_GLOBAL) # Load the GEOPM PMPI wrappers
    import mpi4py.MPI

    def root_print(msg):
        rank = mpi4py.MPI.COMM_WORLD.Get_rank()
        if rank == 0:
            sys.stdout.write(msg)

    def bench_main():
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

    class AppConfig(object):
        def write(self):
            pass

        def get_exec_path(self):
            return os.path.realpath(__file__)

        def get_exec_args(self):
            return ['--bench']


    class TestIntegrationMonitorSleep(unittest.TestCase):
        def setUp(self):
            test_name = 'test_monitor_sleep'
            self._num_node = 4
            self._num_rank = 16
            app_conf = AppConfig()
            agent_conf = geopmpy.io.AgentConf(test_name + '-agent-config.json')
            report_path = test_name + '.report'
            self._launcher = geopm_test_launcher.TestLauncher(app_conf, agent_conf, report_path)
            self._launcher.set_num_node(self._num_node)
            self._launcher.set_num_rank(self._num_rank)
            self._launcher.run(test_name)
            self._output = geopmpy.io.AppOutput(report_path)

        def tearDown(self):
            if sys.exc_info() == (None, None, None) and os.getenv('GEOPM_KEEP_FILES') is None:
                self._output.remove_files()
                self._launcher.remove_files()

        def test_report_exists(self):
            node_names = self._output.get_node_names()
            self.assertEqual(self._num_node, len(node_names))
            for nn in node_names:
                report = self._output.get_report_data(node_name=nn)
                self.assertNotEqual(0, len(report))

    if __name__ == '__main__':
        unittest.main()

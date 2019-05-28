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

"""Test the energy efficient agent by doing a frequency stream/dgemm mix.

"""

import sys

if '--bench' in sys.argv:
    # When --bench is given the script is an MPI application
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
        repeat = 100
        regions = ['stream-unmarked', 'dgemm-unmarked']
        big_o_list = [[1.0 - 0.1 * xx, 0.1 * xx] for xx in range(0, 11)]
        for big_o in big_o_list:
            stream_name = 'stream-{}'.format(big_o[0])
            stream_model = geopmpy.bench.model_region_factory('stream-unmarked', big_o[0], is_verbose)
            dgemm_name = 'dgemm-{}'.format(big_o[1])
            dgemm_model = geopmpy.bench.model_region_factory('dgemm-unmarked', big_o[1], is_verbose)
            region_name = '-'.join((stream_name, dgemm_name)) 
            region_id = geopmpy.prof.region(region_name, geopmpy.prof.REGION_HINT_UNKNOWN)
            for iter in range(repeat):
                geopmpy.prof.enter(region_id)                
                geopmpy.bench.model_region_run(stream_model)
                geopmpy.bench.model_region_run(dgemm_model)
                geopmpy.prof.exit(region_id)
            geopmpy.bench.model_region_delete(dgemm_model)
            geopmpy.bench.model_region_delete(stream_model)

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


    class TestIntegrationEEFreqSweep(unittest.TestCase):
        @classmethod
        def setUpClass(cls):
            """Create launcher, execute this script as a benchmark and store the
            output.

            """
            test_name = 'test_ee_freq_sweep'
            cls._num_node = 4
            cls._num_rank = 16
            app_conf = AppConf()
            agent_conf = geopmpy.io.AgentConf(test_name + '-agent-config.json', 'energy_efficient', {'frequency_min':1.0, 'frequency_max':1.3})
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

    if __name__ == '__main__':
        unittest.main()

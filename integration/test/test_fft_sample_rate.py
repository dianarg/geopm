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

from __future__ import absolute_import

import os
import sys
import unittest
import shlex
import subprocess
import io
import json

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from integration.test import geopm_context
import geopmpy.launcher
import geopmpy.io

#import geopmpy.topo
#import geopmpy.pio

#TODO: run long test?
class TestIntegrationFFTSanity(unittest.TestCase):
    """
    Tests that FFT can run with GEOPM without obvious failures.
    """
    @classmethod
    def setUpClass(cls):
        cls._test_name = 'test_fft_sanity'
        cls._report_path = cls._test_name + '.report'
        cls._trace_path = cls._test_name + '.trace'

        num_node = 1
        num_rank = 16  # TODO: problem size is too big?

        # TODO: detect launcher instead of srun
        argv = ['geopmlaunch', 'srun']
        # TODO: move to apps/fft.py
        integration_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        apps_dir = os.path.join(integration_dir, 'apps')
        exe = os.path.join(apps_dir, 'fft', '.libs', 'nas_ft')
        argv += [exe]
        launcher = geopmpy.launcher.Factory().create(argv,
                                                     job_name=cls._test_name,
                                                     num_node=num_node,
                                                     num_rank=num_rank,
                                                     time_limit=600)
        launcher.run()

    def test_sample_rate(self):
        """
        Check that sample rate is regular and fast.
        """
        # TODO: duplicated code from geopm_test_integration.py
        self._output = geopmpy.io.AppOutput(self._report_path, self._trace_path + '*')
        node_names = self._output.get_node_names()
        self.assertEqual(self._num_node, len(node_names))

        max_mean = 0.01  # 10 millisecond max sample period
        max_nstd = 0.1  # 10% normalized standard deviation (std / mean)
        for nn in node_names:
            tt = self._output.get_trace_data(node_name=nn)
            delta_t = tt['TIME'].diff()
            delta_t = delta_t.loc[delta_t != 0]
            self.assertGreater(max_mean, delta_t.mean())
            # WARNING : The following line may mask issues in the sampling rate. To do a fine grained analysis, comment
            # out the next line and do NOT run on the BSP. This will require modifications to the launcher or manual testing.
            size_orig = len(delta_t)
            delta_t = delta_t[(delta_t - delta_t.mean()) < 3*delta_t.std()]  # Only keep samples within 3 stds of the mean
            self.assertGreater(0.06, 1 - (float(len(delta_t)) / size_orig))
            self.assertGreater(max_nstd, delta_t.std() / delta_t.mean())


if __name__ == '__main__':
    unittest.main()

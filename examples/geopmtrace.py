#!/bin/env python
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

import sys
import time
import geopmpy.pio

'''Example program that runs for 10 minutes and produces a trace file
with user specified signals.

'''

def main(argv):
    if len(argv) == 1:
        sys.stderr.write('Usage: {} SIGNAL_0 [SIGNAL_1 ...]\n'.format(argv[0])
        return

    signal_names = ['TIME']
    signal_names.extend(argv[1:])
    signal_idx = []
    for sn in signal_names:
        signal_idx.append(geopmpy.pio.push_signal(sn, 'board', 0))
    header = '{}\n'.format('|'.join(signal_names))
    sys.stdout.write(header)
    sample_period = 1.0
    num_sample = 600
    for sample_idx in range(num_sample):
        geopmpy.pio.read_batch()
        line = []
        for si in signal_idx:
            line.append(str(geopmpy.pio.sample(si)))
        line = '{}\n'.format('|'.join(line))
        sys.stdout.write(line)
        time.sleep(sample_period)


if __name__ == '__main__':
    main(sys.argv)

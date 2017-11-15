#  Copyright (c) 2015, 2016, 2017, Intel Corporation
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

"""
GEOPM Analysis - Used to run applications and analyze results for specific GEOPM use cases.
"""

import argparse
import sys
import os
import geopmpy

from geopmpy import __version__

from pandas import IndexSlice as idx

def sys_freq_avail():
    step_freq = 100e6
    with open("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_min_freq") as fid:
        min_freq = 1e3 * float(fid.readline())
    with open('/proc/cpuinfo') as fid:
        for line in fid.readlines():
            if line.startswith('model name\t:'):
                sticker_freq = float(line.split('@')[1].split('GHz')[0]) * 1e9
                break
        max_freq = sticker_freq + step_freq

    num_step = 1 + int((max_freq - min_freq) / step_freq)
    result = [step_freq * ss + min_freq for ss in range(num_step)]
    return result

class Analysis(object):
    def __init__(self, name, num_rank, num_node, app_argv):
        self._name = name
        self._num_rank = num_rank
        self._num_node = num_node
        self._app_argv = app_argv
        self._report_paths = []
        self._trace_paths = []
        self._app_output = None

    def launch(self):
        """Run experiment and set data paths corresponding to output.
        """
        raise NotImplementedError('Analysis base class does not implement the launch method()')

    def set_data_paths(self, report_paths, trace_paths=None):
        if self._report_paths is None and self._trace_paths is None:
            self._report_paths = report_paths
            self._trace_paths = trace_paths
        else:
            raise RuntimeError('Analysis object already has report and trace paths populated.')

    def _parse(self):
        if self._app_output is None:
            self._app_output = geopmpy.io.AppOutput(self._report_paths, self._trace_paths)

    def get_report_df(self):
        if self._app_output is not None:
            return self._app_output.get_report_df()

    def get_trace_df(self):
        if self._app_output is not None:
            return self._app_output.get_trace_df()

class MockAnalysis(Analysis):
    def __init__(self, name, num_rank, num_node, app_argv):
        super(MockAnalysis, self).__init__(name, num_rank, num_node, app_argv)
        self._output_path = 'mock_analysis.out'

    def launch(self, geopm_ctl='process', do_geopm_barrier=False):
        with open(self._output_path, 'w') as fid:
            fid.write("This is the mock analysis output.\n")
    def parse(self):
        with open(self._output_path) as fid:
            self._output = fid.read()

    def process(self):
        sys.stdout.write(self._output)
        os.unlink(self._output_path)

class BalancerAnalysis(Analysis):
    pass

class FreqSweepAnalysis(Analysis):
    def __init__(self, name, num_rank, num_node, app_argv):
        super(FreqSweepAnalysis, self).__init__(name, num_rank, num_node, app_argv)

    def launch(self, geopm_ctl='process', do_geopm_barrier=False):
        ctl_conf = geopmpy.io.CtlConf(self._name + '_app.config',
                                      'dynamic',
                                      {'power_budget':400,
                                       'tree_decider':'static_policy',
                                       'leaf_decider':'simple_freq',
                                       'platform':'rapl'})
        if 'GEOPM_SIMPLE_FREQ_RID_MAP' in os.environ:
            del os.environ['GEOPM_SIMPLE_FREQ_RID_MAP']
        if 'GEOPM_SIMPLE_FREQ_ADAPTIVE' in os.environ:
            del os.environ['GEOPM_SIMPLE_FREQ_ADAPTIVE']
        for freq in sys_freq_avail():
            profile_name = self._name + '_freq_{}'.format(freq)
            report_path = os.path.join(self._out_dir, profile_name + '.report')
            self._report_paths.append(report_path)
            if self._app_argv and not os.path.exists(report_path):
                os.environ['GEOPM_SIMPLE_FREQ_MIN'] = str(freq)
                os.environ['GEOPM_SIMPLE_FREQ_MAX'] = str(freq)
                argv = ['dummy', '--geopm-ctl', geopm_ctl,
                                 '--geopm-policy', ctl_conf.get_path(),
                                 '--geopm-report', report_path,
                                 '--geopm-profile', profile_name]
                if do_geopm_barrier:
                    argv.append('--geopm-barrier')
                argv.append('--')
                argv.extend(self._app_argv)
                launcher = geopmpy.launcher.factory(argv, num_rank, num_node)
                launcher.run()
            elif os.path.exists(report_path):
                sys.sterr.write('<geopmpy>: Warning: output file "{}" exists, skipping run.\n'.format(report_path))
            else:
                raise RuntimeError('<geopmpy>: output file "{}" does not exist, but no application was specified.\n'.format(report_path))

    def process(self):
        region_freq_map = self._region_freq_map(self.get_report_df())
        region_freq_str = self._region_freq_str(region_freq_map)
        sys.stdout.write('Region frequency map: {}\n'.format(region_freq_str))

    def _region_freq_map(self, report_df):
        perf_margin = 0.1
        optimal_freq = dict()
        min_runtime = dict()
        is_once = True

        profile_name_list = report_df.index.get_level_values('name').unique().tolist()
        freq_list = [float(pn.split('_freq_')[1]) for pn in profile_name_list if '_freq_' in pn]
        freq_pname = zip(freq_list, profile_name_list)
        freq_pname.sort(reverse=True)

        for freq, profile_name in freq_pname:
            region_mean_runtime = report_df.loc([pandas.IndexSlice[:, profile_name, :, :, :, :, :, :], ].groupby(level='region'))
            for region, region_df in region_mean_runtime:
                runtime = region_df['runtime'].mean()
                if is_once:
                    min_runtime[region] = runtime
                    optimal_freq[region] = freq
                elif min_runtime[region] > runtime:
                    min_runtime[region] = runtime
                    optimal_freq[region] = freq
                elif min_runtime[region] * (1.0 + perf_margin) > runtime:
                    optimal_freq[region] = freq
            is_once = False
        return optimal_freq

    def _region_freq_str(self, region_freq_map):
        result = ['{}:{}'.format(key, value)
                  for (key, value) in region_freq_map.iteritems()]
        result = ','.join(result)
        return result

def main(argv):
    help_str ="""
usage: analysis.py [-h] -t ANALYSIS_TYPE [-o OUTPUT_DIR] [-v] [-l] [--version]
                   [APP_ARGV [APP_ARGV ...]]

GEOPM Analysis - Used to run applications and analyze results for specific
                 GEOPM use cases.

required:
  -t ANALYSIS_TYPE, --analysis_type ANALYSIS_TYPE
                        type of analysis to perform, select from: freq_sweep,
                        balancer
positional arguments:
  APP_ARGV              positional arguments are the application and its
                        arguments. (default: None)

optional arguments:
  -h, --help            show this help message and exit
  -o OUTPUT_DIR, --output_dir OUTPUT_DIR
                        the output directory for the generated files (default: '.')
  -v, --verbose         print debugging information. (default: False)
  -l, --launch_only     run application and generate data, do not analyze the
                        data. (default: False)
  --version             show program's version number and exit

"""
    version_str = """\
GEOPM version {}
Copyright (C) 2015, 2016, 2017, Intel Corporation. All rights reserved.
""".format(__version__)

    if '--help' in argv or '-h' in argv:
        sys.stdout.write(help_str)
        exit(0)
    if '--version' in argv:
        sys.stdout.write(version_str)
        exit(0)

    analysis_type_map = {'freq_sweep':FreqSweepAnalysis,
                         'balancer':BalancerAnalysis,
                         'test':MockAnalysis}
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter, add_help=False)

    parser.add_argument('-t', '--analysis_type',
                           help='type of analysis to perform, select from: {}'.format(', '.join(analysis_type_map.keys())),
                           action='store', required=True, default='REQUIRED OPTION')
    parser.add_argument('app_argv', metavar='APP_ARGV',
                        action='store', nargs='*')
    parser.add_argument('-o', '--output_dir',
                        action='store', default='.')
    parser.add_argument('-v', '--verbose',
                        action='store_true')
    parser.add_argument('-l', '--launch_only',
                        action='store_true')
    parser.add_argument('-p', '--profile_name_prefix',
                        action='store', default=None)
    parser.add_argument('-n', '--num_rank',
                        action='store', default=None, type=int)
    parser.add_argument('-N', '--num_node',
                        action='store', default=None, type=int)

    args = parser.parse_args(argv)
    if args.analysis_type not in analysis_type_map:
        raise SyntaxError('Analysis type: "{}" unrecognized.'.format(args.analysis_type))
    analysis = analysis_type_map[args.analysis_type](args.profile_name_prefix, args.num_rank, args.num_node, args.app_argv)
    analysis.launch()
    if not args.launch_only:
        analysis.parse()
        analysis.process()


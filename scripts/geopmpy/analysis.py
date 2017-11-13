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
    def __init__(self, name, app_argv, num_rank, num_node)
        self._name = name
        self._app_argv = app_argv
        self._num_rank = num_rank
        self._num_node = num_node
        self._report_paths = []
        self._trace_paths = []
        self._app_output = None

    def launch(self):
        raise NotImplementedError('Analysis base class does not implement the launch method()')

    def set_output(self, report_paths, trace_paths=None):
        if self._report_paths is None and self._trace_paths is None:
            self._report_paths = report_paths
            self._trace_paths = trace_paths
        else:
            raise RuntimeError('Analysis object already has report and trace paths populated.')

    def parse(self):
        self._app_output = geopmpy.io.AppOutput(self._report_paths, self._trace_paths)

    def report_df(self):
        return self._app_output.report_df()

    def get_report_paths(self):
        return self._report_paths

    def set_report_paths(self, paths):
        self._report_paths = report_paths

    def append_report_paths(self, path):
        self._report_paths.append(path)

    def extend_report_paths(self, paths):
        self._report_paths.extend(paths)

class FreqSweepAnalysis(Analysis):
    def __init__(self, name, app_argv, num_rank, num_node):
        super(FreqSweepAnalysis, self).__init__(name, app_argv, num_rank, num_node)

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
            report_path = profile_name + '.report'
            self.append_report_paths(report_path)
            if not os.path.exists(report_path):
                os.environ['GEOPM_SIMPLE_FREQ_MIN'] = str(freq)
                os.environ['GEOPM_SIMPLE_FREQ_MAX'] = str(freq)
                argv = ['dummy', '--geopm-ctl', geopm_ctl,
                                 '--geopm-policy', ctl_conf.get_path(),
                                 '--geopm-report', report_path,
                                 '--geopm-profile', profile_name]
                if do_geopm_barrier:
                    argv.append('--geopm-barrier')
                argv.append('--')
                argv.extend(app_argv)
                launcher = geopmpy.launcher.factory(argv, num_rank, num_node)
                launcher.run()
            else:
                sys.sterr.write('<geopmpy>: Warning: output file "{}" exists, skipping run.\n'.format(report_path)

def ee_region_freq_map(report_paths):
    optimal_freq = dict()
    min_runtime = dict()
    is_once = True
    for report_path in report_paths:
        # FIXME NEED TO DETERMINE FREQUENCY EACH REPORT WAS GENERATED
        # FOR AND SORT IN INVERSE ORDER.
        region_mean_runtime = geopmpy.io.AppOutput(report_path).get_report_df().groupby('region')['runtime'].mean()
        for region in region_mean_runtime.keys():
            if is_once:
                min_runtime[region] = region_mean_runtime[region]
                optimal_freq[region] = freq
            elif min_runtime[region] > region_mean_runtime[region]:
                min_runtime[region] = region_mean_runtime[region]
                optimal_freq[region] = freq
            elif min_runtime[region] * (1 + perf_margin) > region_mean_runtime[region]:
                optimal_freq[region] = freq
        is_once = False
    return optimal_freq

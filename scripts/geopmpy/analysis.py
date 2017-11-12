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


def sys_avail_freq():
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

def launch_freq_sweep(freq_sweep, app_argv, name_base):
    report_paths = []
    options = dict()
    options['power_budget'] = 400 # Run at TDP to ensure RAPL does not win.
    options['tree_decider'] = 'static_policy'
    options['leaf_decider'] = 'simple_freq'
    options['platform'] = 'rapl'
    mode = 'dynamic'
    ctl_conf = geopmpy.io.CtlConf(name_base + '_app.config', mode, options)

    for freq in freq_sweep:
        profile_name = name_base + '_freq_{}'.format(freq)
        report_path = profile_name + '.report'
        report_paths.append(report_path)
        try:
            os.stat(report_path)
        except OSError:
            if ctl_conf._options['leaf_decider'] != 'simple_freq':
                raise RuntimeError('Leaf decider must be "simple_freq"')
            os.environ['GEOPM_SIMPLE_FREQ_MIN'] = str(freq)
            os.environ['GEOPM_SIMPLE_FREQ_MAX'] = str(freq)
            if 'GEOPM_SIMPLE_FREQ_RID_MAP' in os.environ:
                del os.environ['GEOPM_SIMPLE_FREQ_RID_MAP']
            if 'GEOPM_SIMPLE_FREQ_ADAPTIVE' in os.environ:
                del os.environ['GEOPM_SIMPLE_FREQ_ADAPTIVE']

            argv = ['dummy', '--geopm-ctl', geopm_ctl,
                             '--geopm-policy', ctl_conf.get_path(),
                             '--geopm-report', report_path,
                             '--geopm-profile', profile_name]
            if do_region_barrier:
                argv.append('--geopm-barrier')
            argv.append('--')
            argv.extend(app_argv)
            launcher = geopmpy.launcher.factory(argv, num_rank, num_node)

            sys.stderr.write(79 * '-' + '\n')
            sys.stderr.write('Mix ratio index {}\nDGEMM:STREAM = {}:{}\n'.format(ratio_idx, ratio[0], ratio[1]))

            sys.stderr.write('\nCtl config -\n{}\n'.format(ctl_conf))
            sys.stderr.write('\nFrequency: {}\n'.format(freq))
            launcher.run()
    return report_paths

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

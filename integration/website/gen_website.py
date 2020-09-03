#!/usr/bin/env python
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
import os
import textwrap
import glob
import yattag
import argparse


def add_license(doc):
    license_text = textwrap.dedent(''' \
    <!DOCTYPE html>
    <!--
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
      -->
    ''')
    doc.asis(license_text)


def add_head(doc):
    with doc.tag('head'):
        doc.asis('<meta name="viewport" content="width=device-width, initial-scale=1">')
        with doc.tag('style'):
            # TODO: indentation looks funky here
            stylesheet = textwrap.dedent('''
            img {
            display: block;
            margin-left: auto;
            margin-right: auto;
            border:1px solid black;
            }
            h2 {text-align: center; font-family: sans-serif;}
            .note {max-width: 750px; margin: 0 auto;}
            h3 {text-align: center;}
            table, th, td {border:1px solid black; border-collapse: collapse;}
            th, td {padding: 5px;}
            ''')
            doc.asis(stylesheet)


def make_toc(doc, items):
    with doc.tag('div', klass='note'):
        for link_name in items:
            with doc.tag('a', href='#{}'.format(link_name)):
                doc.text(link_name)
            doc.stag('br')


class ResultInfo:
    def __init__(self, path, description):
        self.path = path
        self.description = description


def benchmark_results(doc, bench_name, result_dirs):
    doc.line('h3', bench_name, id=bench_name.lower())
    with doc.tag('div', klass='note'):
        for result in result_dirs:
            rdir = result.path
            # discover text tables
            # TODO: super brittle
            stat_logs = glob.glob(os.path.join(rdir, '*_stats.log'))
            # discover plots in figures subdir
            images = glob.glob(os.path.join(rdir, 'figures', '*.png'))
            with doc.tag('p'):
                with doc.tag('b'):
                    doc.text('{}: '.format(rdir))
                with doc.tag('a', href=rdir):
                    doc.text('raw data')
            doc.line('p', result.description)
            for log in stat_logs:
                with doc.tag('a', href=log):
                    doc.text(os.path.basename(log))
                doc.stag('br')
            for img in images:
                doc.stag('img', src=img)
            doc.stag('br')


class SmokeStatus:
    def __init__(self, path, run_status, analysis_status):
        self.path = path
        self.run_status = run_status
        self.analysis_status = analysis_status


def weekly_status(doc):
    experiments = ['monitor', 'power_sweep', 'frequency_sweep']
    apps = ['dgemm_tiny', 'dgemm', 'nekbone', 'minife', 'gadget', 'amg', 'hpcg']
    # TODO: currently evaluated by hand; load from disk
    status = {
              ('frequency_sweep', 'dgemm_tiny'): SmokeStatus('smoke/49667_dgemm_tiny_frequency_sweep', 'PASS', 'PASS'),
              ('frequency_sweep', 'dgemm'): SmokeStatus('smoke/49667_dgemm_frequency_sweep', 'PASS', 'PASS'),
              ('frequency_sweep', 'minife'): SmokeStatus('smoke/49667_minife_frequency_sweep', 'PASS', 'PASS'),
              ('frequency_sweep', 'nekbone'): SmokeStatus('smoke/49667_nekbone_frequency_sweep', 'PASS', 'PASS'),
              ('monitor', 'dgemm_tiny'): SmokeStatus('smoke/49668_dgemm_tiny_monitor', 'PASS', 'PASS'),
              ('monitor', 'dgemm'): SmokeStatus('smoke/49668_dgemm_monitor', 'PASS', 'PASS'),
              ('monitor', 'minife'): SmokeStatus('smoke/49668_minife_monitor', 'PASS', 'PASS'),
              ('monitor', 'nekbone'): SmokeStatus('smoke/49668_nekbone_monitor', 'PASS', 'PASS'),
              ('power_sweep', 'dgemm_tiny'): SmokeStatus('smoke/49669_dgemm_tiny_power_sweep', 'PASS', 'PASS'),
              ('power_sweep', 'dgemm'): SmokeStatus('smoke/49669_dgemm_power_sweep', 'PASS', 'PASS'),
              ('power_sweep', 'minife'): SmokeStatus('smoke/49669_minife_power_sweep', 'PASS', 'PASS'),
              ('power_sweep', 'nekbone'): SmokeStatus('smoke/49669_nekbone_power_sweep', 'PASS', 'PASS'),
    }
    with doc.tag('div', klass='note'):
        doc.line('h3', 'Integration smoke tests')
        doc.line('p', 'Check that: 1) run scripts produce the expected reports; ' +
                      '2) reports can be used with analysis scripts for the experiment type to produce the expected plots and tables. ' +
                      'No evaluation of performance.')
        # TODO: pandas df to table
        with doc.tag('table'):
            #doc.stag('col')
            # header row: experiment type
            with doc.tag('tr'):
                doc.stag('td', rowspan=2)
                for exp in experiments:
                    doc.line('th', exp, colspan=3)
            # header row: test type
            with doc.tag('tr'):
                for exp in experiments:
                    doc.line('th', 'run script')
                    doc.line('th', 'analysis')
                    doc.line('th', 'raw data')
            for app in apps:
                with doc.tag('tr'):
                    doc.line('td', app)
                    for exp in experiments:
                        if (exp, app) in status:
                            doc.line('td', status[(exp, app)].run_status)
                            doc.line('td', status[(exp, app)].analysis_status)
                            with doc.tag('td'):
                                doc.line('a', 'data', href=status[(exp, app)].path)
                        else:
                            doc.line('td', 'not run', colspan=3, style='text-align: center;')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true', default=False)
    args = parser.parse_args()

    doc = yattag.Doc()
    add_license(doc)
    with doc.tag('html'):
        add_head(doc)
        with doc.tag('body'):
            make_toc(doc, ['nekbone', 'minife'])
            nekbone_jobs = [
                ResultInfo(os.path.join('data', 'power_balancer_energy_nekbone_v1.1.0-470-ge978df6_2020-08-27-1598515356'),
                           'Nekbone with power balancer, 12 nodes, 5 iterations'),
                ResultInfo(os.path.join('data','49727_nekbone_energy_efficiency'),
                           'Nekbone with inserted MPI_Barriers, 8 nodes, 10-12 iterations'),
                # ResultInfo(os.path.join('data', '49666_nekbone_energy_efficiency'),
                #            'Nekbone with inserted MPI_Barriers, sweeping over frequency setting for barrier\n12 nodes, 5 iterations'),
                # ResultInfo(os.path.join('data', '49676_nekbone_energy_efficiency'),
                #            '12 nodes, 12 iterations'),
            ]
            minife_jobs = [
                #ResultInfo(os.path.join('data', '49678_minife_power_sweep'), 'MiniFE power sweep, 8 nodes. 5 iterations'),
                ResultInfo(os.path.join('data', 'power_balancer_energy_minife_v1.1.0-485-g2365b27_2020-08-31-1598915009'),
                           'MiniFE power sweep, 8 nodes, 20 iterations'),
                ResultInfo(os.path.join('data', '49677_minife_frequency_sweep'), 'MiniFE frequency sweep, 8 nodes, 5 iterations'),
            ]
            benchmark_results(doc, 'Nekbone', nekbone_jobs)
            benchmark_results(doc, 'MiniFE', minife_jobs)
            weekly_status(doc)

    result = yattag.indent(doc.getvalue())
    debug = True
    if args.debug:
        sys.stdout.write(result + '\n')
    with open('index.html', 'w') as outfile:
        outfile.write(result)

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

import matplotlib.pyplot as plt
from geopmpy.io import RawReportCollection
import glob

def main():
    reports = glob.glob('*.report')
    rrc = RawReportCollection(reports)
    df = rrc.get_app_df()
    profile_names = df['Profile'].unique()
    profile_names.sort()
    pte_list = []
    for prof in profile_names:
        temp_df = df.loc[df['Profile'] == prof]
        power = temp_df['POWER_PACKAGE_LIMIT_TOTAL']
        power = power.unique()[0]
        time = temp_df['runtime (sec)']
        time = time.sum() / len(time)
        energy = temp_df['package-energy (joules)']
        energy = energy.sum() / len(energy)
        pte_list.append((power, time, energy))

    power_limits = df['POWER_PACKAGE_LIMIT_TOTAL'].unique()
    power_limits.sort()
    pt = {pl : 0 for pl in power_limits}
    pe = {pl : 0 for pl in power_limits}
    pn = {pl : 0 for pl in power_limits}
    for pte in pte_list:
        pt[pte[0]] += pte[1]
        pe[pte[0]] += pte[2]
        pn[pte[0]] += 1
    for pl in power_limits:
        pt[pl] = pt[pl] / pn[pl]
        pe[pl] = pe[pl] / pn[pl]
    plot_dict = dict()
    pkey = 'Power limit reduction from TDP (watts)'
    tkey = 'time increase'
    ekey = 'energy reduction'
    plot_dict[pkey] = []
    plot_dict[tkey] = []
    plot_dict[ekey] = []
    ref = power_limits[-1]
    for pl in power_limits:
        plot_dict[pkey].append(ref - pl)
        tf = 100 * (pt[pl] / pt[ref] - 1.0)
        ef = 100 * (1 - pe[pl] / pe[ref])
        plot_dict[tkey].append(tf)
        plot_dict[ekey].append(ef)

    plt.title('Unified Model power sweep')
    plt.plot(plot_dict[pkey], plot_dict[tkey], 'bo-', label=tkey)
    plt.plot(plot_dict[pkey], plot_dict[ekey], 'ro-', label=ekey)

    plt.xlabel(pkey)
    plt.ylabel('percentage change (%)')
    plt.legend()
    plt.savefig('unifiedmodel-power-sweep.png')
    plt.show()

if __name__ == '__main__':
    main()

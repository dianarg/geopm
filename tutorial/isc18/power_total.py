#!/usr/bin/env python

import sys
import pandas
import geopmpy.io


def print_power_consumption(report_path, trace_path):
    ''' From test_power_consumption integration test. Only works with Agent code path.'''
    output = geopmpy.io.AppOutput(report_path, trace_path + '*')
    node_names = output.get_node_names()
    # Total power consumed will be Socket(s) + DRAM
    for nn in node_names:
        tt = output.get_trace(nn)

        epoch = '0x8000000000000000'

        first_epoch_index = tt.loc[tt['region_id'] == epoch][:1].index[0]
        epoch_dropped_data = tt[first_epoch_index:]  # Drop all startup data

        power_data = epoch_dropped_data.filter(regex='energy')
        power_data['seconds'] = epoch_dropped_data['seconds']
        power_data = power_data.diff().dropna()
        power_data.rename(columns={'seconds': 'elapsed_time'}, inplace=True)
        power_data = power_data.loc[(power_data != 0).all(axis=1)]  # Will drop any row that is all 0's

        pkg_energy_cols = [s for s in power_data.keys() if 'pkg_energy' in s]
        dram_energy_cols = [s for s in power_data.keys() if 'dram_energy' in s]
        power_data['socket_power'] = power_data[pkg_energy_cols].sum(axis=1) / power_data['elapsed_time']
        power_data['dram_power'] = power_data[dram_energy_cols].sum(axis=1) / power_data['elapsed_time']
        power_data['combined_power'] = power_data['socket_power'] + power_data['dram_power']

        pandas.set_option('display.width', 100)
        print 'Power stats from {} :\n{}'.format(nn, power_data.describe())


if __name__ == '__main__':
    power_cap = "200"
    if len(sys.argv) == 2:
        power_cap = sys.argv[1]
    # look in current directory
    print_power_consumption('isc18_governor_{}.report'.format(power_cap),
                            'isc18_governor_{}.trace'.format(power_cap))

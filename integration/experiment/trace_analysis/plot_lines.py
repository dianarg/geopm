#!/usr/bin/env python

import os
import sys
import glob
import hashlib
import argparse
import pandas
import matplotlib.pyplot as plt


# This could use io.py AppOutput, but here is the start of a refactoring

# TODO: no concept of output_dir here
# TODO: do_cache input
def cheque_cash(filenames, parse_function, hdf_load_function, hdf_save_function,
                verbose=True):
    '''Load data from files using the given parse_function, or from an
       HDF5 cache file if the cache is newer than all the input files.

    Inputs:
    - filenames: list of file paths or glob pattern
    - parse_function: function that takes in a list of filepaths and
                      returns data in a form defined by the caller of
                      this function
    - hdf_load_function: function that takes in a path to an hdf5 file
                         and returns data in a form defined by the
                         caller of this function
    - hdf_save_function: function that takes in the data from parse
                         and saves an hdf5 at the given path.  The
                         format should be compatible with future calls
                         to hdf_load_function().
    - verbose: enable to print debug information to stdout

    '''
    # convert input to list of string paths
    if type(filenames) is list:
        paths = filenames
    else:
        try:
            paths = glob.glob(filenames)
        except:
            raise RuntimeError("<geopm> filenames must be a list of paths or a glob pattern {}".format(filenames))
    paths_str = str(paths)
    if len(paths) == 0:
        raise RuntimeError('<geopm> No files found with pattern {}.'.format(filenames))

    # create h5 file name
    # TODO: should also encode the source of data to fix #1258
    try:
        h5_id = hashlib.shake_256(paths_str.encode()).hexdigest(14)
    except AttributeError:
        h5_id = hash(paths_str)
    h5_name = 'data_{}.h5'.format(h5_id)

    # check if cache is older than files
    if os.path.exists(h5_name):
        cache_mod_time = os.path.getmtime(h5_name)
        regen_cache = False
        for path in paths:
            mod_time = os.path.getmtime(path)
            if mod_time > cache_mod_time:
                regen_cache = True
        if regen_cache:
            os.remove(h5_name)

    # attempt to load data from h5 file.  will fail if not present
    data = None
    try:
        data = hdf_load_function(h5_name)  # pandas.read_hdf(h5_name, 'data')
        if verbose:
            sys.stdout.write('Loaded data from {}.\n'.format(h5_name))
    except IOError:
        sys.stderr.write('Warning: <geopm> geopmpy.io: HDF5 cache file not detected or older than files.  Data will be saved to {}.\n'
                         .format(h5_name))
        data = parse_function(paths)
        hdf_save_function(data, h5_name)
        if verbose:
            sys.stdout.write('Data saved to {}.\n'.format(h5_name))

    return data


def parse_traces(traces):
    df = pandas.DataFrame()
    for trace in traces:
        try:
            with open(trace) as infile:
                df = df.append(pandas.read_csv(infile, delimiter='|', comment='#'))
        except:
            raise RuntimeError("<geopm> Failed to parse trace file {}".format(trace))
    return df


def load_traces(h5_path):
    df = pandas.read_hdf(h5_path, 'data')
    return df


def save_traces(data, h5_path):
    data.to_hdf(h5_path, 'data')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--columns', type=str, required=True)
    parser.add_argument('trace_files', nargs='+')

    args = parser.parse_args()

    columns = args.columns.split(',')
    if len(columns) == 0:
        sys.stderr.write('Columns required.')
        sys.exit(1)
    sys.stdout.write('Plotting columns: {}\n'.format(args.columns))

    for trace in args.trace_files:
        df = cheque_cash(trace, parse_traces, load_traces, save_traces)
        for column in columns:
            print(df[column].describe())

            plt.plot(df[column], label=trace.split('-')[-1]+'-'+column)

    plt.title(', '.join(columns))
    plt.legend()
    plt.savefig('test.png')

# -*- coding: utf-8 -*-
"""
Created on Thu May 28 16:30:09 2020

@author: AmP
"""

import csv
import numpy as np


try:
    from os import scandir
except ImportError:
    from scandir import scandir


def read_csv(filename):
    dic = {}
    mapping = {}
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter='\t')
        for idx, row in enumerate(reader):
            # print ', '.join(row)
            if idx == 0:
                for jdx, key in enumerate(row):
                    mapping[jdx] = key
                    dic[key] = []
            else:
                for jdx, val in enumerate(row):
                    try:
                        val_ = float(val)
                    except ValueError:
                        val_ = np.nan
                    dic[mapping[jdx]].append(val_)
    return dic


def _get_csv_names(dirpath):
    for entry in scandir(dirpath):
        if entry.is_file() and entry.name.endswith('.csv'):
            yield entry.name


def get_csv_set(dirpath):
    names = _get_csv_names(dirpath)
    return [name.split('.csv')[0] for name in sorted(names)]


def load_data(path, sets=['00']):
    dataBase = []

    xscale = 112./1000  # after changing resolution of RPi
    xshift = -12  # cm
    yshift = -45  # cm

    for exp in sets:
        data = read_csv(path+"{}.csv".format(exp))
        print([key for key in data])

        try:
            start_idx = data['f0'].index(1)  # upper left foot attached 1sttime
        except ValueError:  # no left foot is fixed
            start_idx = 0
        start_time = data['time'][start_idx]
        data['time'] = \
            [round(data_time - start_time, 3) for data_time in data['time']]
        for key in data:
            if key[0] in ['x', 'y']:
                shift = xshift if key[0] == 'x' else yshift
                data[key] = [i*xscale + shift for i in data[key]]

        dataBase.append(data)

    return dataBase

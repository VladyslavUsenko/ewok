#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np


mapping_data = np.loadtxt('mapping_time.txt')
opt_data = np.loadtxt('optimization_time.txt')

print 'mapping time: computing pointcloud, moving volume, inserting measurements [ms]'
print mapping_data.mean(axis=0)/1e6

print 'sdf computation, optimization time [ms]'
print opt_data.mean(axis=0)/1e6

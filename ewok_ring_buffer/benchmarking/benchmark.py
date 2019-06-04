#!/usr/bin/env python

import urllib
import os
import tarfile
import associate

import matplotlib.pyplot as plt
import numpy as np

dataset_url = 'https://vision.in.tum.de/rgbd/dataset/freiburg2/rgbd_dataset_freiburg2_pioneer_slam.tgz'
filename_zip = 'rgbd_dataset_freiburg2_pioneer_slam.tgz'
filename = 'rgbd_dataset_freiburg2_pioneer_slam'

# go to benchmak directory
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

if not os.path.exists(filename_zip):
    print 'Downloading dataset file ', filename_zip
    testfile = urllib.URLopener()
    testfile.retrieve(dataset_url, filename_zip)

if not os.path.exists(filename):
    print 'Extracting dataset ', filename
    tar = tarfile.open(filename_zip, "r:gz")
    tar.extractall()
    tar.close()

if not os.path.exists(filename + '/depth_gt.txt'):
    first_list = associate.read_file_list(filename + '/depth.txt')
    second_list = associate.read_file_list(filename + '/groundtruth.txt')

    matches = associate.associate(first_list, second_list, 0.0, 0.02)

    f = open(filename + '/depth_gt.txt', 'w')
    for a,b in matches:
        f.write("%f %s %f %s\n"%(a, " ".join(first_list[a]), b-0.0, " ".join(second_list[b])))
    f.close()

print 'Dataset is ready.'

os.system('rosrun ewok_ring_buffer tum_rgbd_ring_buffer_example rgbd_dataset_freiburg2_pioneer_slam res.txt')

print 'Results are ready.'

data = np.loadtxt('res.txt')

plt.figure()
plt.hist(data[:,0]/1e6, 100)
plt.figure()
plt.hist(data[:,1]/1e6, 100)
plt.figure()
plt.hist(data[:,2]/1e6, 100)
plt.show()

print 'Done.'


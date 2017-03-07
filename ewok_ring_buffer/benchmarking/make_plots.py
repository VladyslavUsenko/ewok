#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np


data = np.loadtxt('res.txt')

plt.rcParams.update({'font.size': 22})

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

f_size = (8,7)

plt.figure(figsize=f_size)
plt.hist(data[:,0]/1e6, 100)
plt.title("Ring buffer")
plt.xlabel("Insertion time [ms]")
plt.savefig("ring_buffer_hist.pdf")

plt.figure(figsize=f_size)
plt.hist(data[:,2]/1e6, 100)
plt.title("Octomap")
plt.xlabel("Insertion time [ms]")
plt.savefig("octomap_hist.pdf")


def compute_limit_func(x, l):
    y = np.zeros_like(x)
    y[x > l] = np.exp(x[x>l]**2 - l**2)
    return y

l1 = 3.0
l2 = 6.0
l3 = 9.0

x = np.arange(0, 10, 0.01)
y1 = compute_limit_func(x, l1)
y2 = compute_limit_func(x, l2)
y3 = compute_limit_func(x, l3)

plt.figure(figsize=f_size)
plt.plot(x, y1, 'r')
plt.plot(x, y2, 'g')
plt.plot(x, y3, 'b')

x1,x2,y1,y2 = plt.axis()
plt.axis((x1,x2,0,1000))

plt.title("Soft Limit Cost Function")
plt.savefig("soft_limit.pdf")

print 'Done.'


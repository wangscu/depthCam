'''
==============
3D scatterplot
==============

Demonstration of a basic scatterplot in 3D.
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import pcl

point = pcl.load("pointcloud.pcd").to_array()


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

xs = point[:, 0:1]
ys = point[:, 1:2]
zs = point[:, 2:3]

ax.scatter(xs, ys, zs, c='r', marker='o')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
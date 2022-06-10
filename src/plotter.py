# Liv plotter for ROS Course 
# Author: Del Piero Flores
# File: live_plotter

import matplotlib.pyplot as plt
from matplotlib import style

fig = plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)


graph_data = open('data/position.txt','r').read()
lines = graph_data.split('\n')
xd = 0
yd = 0

xs = []
ys = []

phid_s = []
phi_s = []
t_s = []
for line in lines:
    if len(line) > 1:
        t, xd, yd, phid, x, y, phi = line.split(',')
        t_s.append(float(t))
        xs.append(float(x))
        ys.append(float(y))
        phid_s.append(float(phid))
        phi_s.append(float(phi))


ax1.plot(float(xd), float(yd), marker="*", markersize=20, markeredgecolor="red", markerfacecolor="red")
ax1.plot(xs, ys, 'b-')

ax2.plot(t_s, phid_s, 'r-')
ax2.plot(t_s, phi_s, 'b-')

plt.show()
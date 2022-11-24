# Liv plotter for ROS Course 
# Author: Del Piero Flores
# File: live_plotter

import matplotlib.pyplot as plt
import os

POSITION_OUTPUT_FILE = os.path.dirname(os.path.realpath(__file__)) + "/data/position.txt"

fig = plt.figure()
ax1 = fig.add_subplot(2,2,2)
ax2 = fig.add_subplot(2,2,1)
ax3 = fig.add_subplot(2,2,3)


graph_data = open(POSITION_OUTPUT_FILE,'r').read()
lines = graph_data.split('\n')
xd = 0
yd = 0

xs = []
ys = []

phid_s = []
phi_s = []
t_s = []
error_s = []
for line in lines:
    if len(line) > 1:
        t, xd, yd, phid, x, y, phi, error = line.split(',')
        t_s.append(float(t))
        xs.append(float(x))
        ys.append(float(y))
        phid_s.append(float(phid))
        phi_s.append(float(phi))
        error_s.append(float(error))



ax1.plot(float(xd), float(yd), marker="*", markersize=20, markeredgecolor="red", markerfacecolor="red")
ax1.plot(xs, ys, 'b-')
ax1.set_xlabel("x position (m)")
ax1.set_ylabel("y position (m)")

ax2.plot(t_s, phid_s, 'r-')
ax2.plot(t_s, phi_s, 'b-')
ax2.set_xlabel("time (s)")
ax2.set_ylabel("Heading angle (rad)")

ax3.plot(t_s, error_s, 'g-')
ax3.set_xlabel("time (s)")
ax3.set_ylabel("error")

plt.show()


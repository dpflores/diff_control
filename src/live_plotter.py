# Liv plotter for ROS Course 
# Author: Del Piero Flores
# File: live_plotter

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(3,1,1)
ax2 = fig.add_subplot(3,1,2)
ax3 = fig.add_subplot(3,1,3)

def animate(i):
    graph_data = open('data/position.txt','r').read()
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

    ax1.clear()
    ax1.plot(float(xd), float(yd), marker="*", markersize=20, markeredgecolor="red", markerfacecolor="red")
    ax1.plot(xs, ys, 'b-')

    ax2.clear()
    ax2.plot(t_s, phid_s, 'r-')
    ax2.plot(t_s, phi_s, 'b-')

    ax3.clear()
    ax3.plot(t_s, error_s, 'g-')

ani = animation.FuncAnimation(fig, animate, interval=10)
plt.show()
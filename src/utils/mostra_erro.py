# -*- coding: utf-8 -*-
from extract_data import ExtractData
from math import degrees
parser = ExtractData()
parser.ExtractError()

def parse():
    error = open("errors.csv")
    X = []
    Y = []
    ANGLE = []
    for line in error:
        data = line.split(",")
        X.append(float(data[0]))
        Y.append(float(data[1]))
        ANGLE.append(float(data[2]))
    return X,Y,ANGLE

x_error, y_error, angle_error = parse()
import pylab
t = [i*0.2 for i in range(0,len(x_error))]


pylab.plot(t,x_error,'r')
pylab.plot(t,y_error,'g')

ax1 = pylab.gca()
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Distance error [cm]")
pylab.legend((r'$x$', r'$y$'), shadow = True, loc = (0.84, 0.84))
ltext = pylab.gca().get_legend().get_texts()
pylab.setp(ltext[0], fontsize = 14, color = 'r')
pylab.setp(ltext[1], fontsize = 14, color = 'g')
pylab.grid(True)

ax2 = ax1.twinx()
ax2.set_ylabel('axis2', color='b')
ax2.plot(t,map(degrees,angle_error), color='b')
ax2.set_ylabel("Orientation error [degrees]")
pylab.legend((r"$\psi$",), shadow = True, loc = (0.84, 0.76))
ltext = pylab.gca().get_legend().get_texts()
pylab.setp(ltext[0], fontsize = 14, color = 'b')

pylab.grid(True)

for tl in ax2.get_yticklabels():
    tl.set_color('b')

pylab.show()

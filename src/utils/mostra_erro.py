# -*- coding: utf-8 -*-
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
pylab.plot(t,x_error)
pylab.plot(t,y_error)
#pylab.plot(t,angle_error)
pylab.show()

# -*- coding: utf-8 -*-
from extract_data import ExtractData
parser = ExtractData()
parser.ExtractPlanSpeeds()
parser.ExtractOdomSpeeds()

def parse(filename):
    error = open(filename)
    V = []
    W = []
    for line in error:
        data = line.split(",")
        V.append(float(data[0]))
        W.append(float(data[1]))
    return V,W
    
v, w = parse("plan_speeds.csv")
vr, wr = parse("odom_speeds.csv")
import pylab
t = [i*0.2 for i in range(0,len(v))]
pylab.plot(t, v)
pylab.plot(t, vr)
pylab.grid(True)
fig = pylab.figure()
pylab.plot(t, w)
pylab.plot(t, wr)
pylab.grid(True)
pylab.show()

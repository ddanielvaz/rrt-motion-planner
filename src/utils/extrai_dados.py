# -*- coding: utf-8 -*-
data = open("data.log","r")
error = open("errors.csv","w")
for line in data:
    if line.startswith("x_diff:"):
        a = line.replace("\n","").replace(":", " ").split(" ")
        # pega apenas strings que contenham algum caracter
        s = [x for x in a if x]
        x, y, angle = float(s[1]), float(s[3]), float(s[5])
        error.write("%f,%f,%f\n"%(x,y,angle))
error.close()
data.close()
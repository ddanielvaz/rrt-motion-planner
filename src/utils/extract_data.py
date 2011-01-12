# -*- coding: utf-8 -*-
class ExtractData(object):
    def __init__(self):
        print "ExtractData instanced."

    def ExtractError(self):
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
        print "Extracting error complete."

    def ExtractPlanSpeeds(self):
        data = open("data.log","r")
        plan_speeds = open("plan_speeds.csv","w")
        for line in data:
            if line.startswith("vx_path:"):
                a = line.replace("\n","").replace(":", " ").split(" ")
                # pega apenas strings que contenham algum caracter
                s = [x for x in a if x]
                v, w = float(s[1]), float(s[3])
                plan_speeds.write("%f,%f\n"%(v,w))
        plan_speeds.close()
        data.close()
        print "Extracting planned speeds complete."

    def ExtractDiffSpeeds(self):
        data = open("data.log","r")
        diff_speeds = open("diff_speeds.csv","w")
        for line in data:
            if line.startswith("v_diff:"):
                a = line.replace("\n","").replace(":", " ").split(" ")
                # pega apenas strings que contenham algum caracter
                s = [x for x in a if x]
                v, w = float(s[1]), float(s[3])
                diff_speeds.write("%f,%f\n"%(v,w))
        diff_speeds.close()
        data.close()
        print "Extracting diff speeds complete."
        
    def ExtractOdomSpeeds(self):
        data = open("data.log","r")
        odom_speeds = open("odom_speeds.csv","w")
        for line in data:
            if line.startswith("vx_robot:"):
                a = line.replace("\n","").replace(":", " ").split(" ")
                # pega apenas strings que contenham algum caracter
                s = [x for x in a if x]
                if len(s) > 4:
                    v, w = float(s[1]), float(s[-1])
                else:
                    v, w = float(s[1]), float(s[3])
                odom_speeds.write("%f,%f\n"%(v,w))
        odom_speeds.close()
        data.close()
        print "Extracting odom speeds complete."

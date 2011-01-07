# -*- coding: utf-8 -*-

fp = open("log.statistics", "r")

total_time = 0.0
not_reached = 0.0
n_reached = 0.0
avg_time_reached = 0.0
avg_trial_reached = 0.0
avg_node_reached = 0.0
avg_time_not_reached = 0.0
avg_total_time = 0.0

for line in fp:
    line = line.strip()
    pertime,trials,added_nodes,reached = map(float,line.split(";"))
    #print pertime,trials,added_nodes,reached
    total_time += pertime
    if reached:
        n_reached += 1.0
        avg_time_reached += pertime
        avg_trial_reached += trials
        avg_node_reached += added_nodes
    else:
        not_reached += 1.0
        avg_time_not_reached += pertime

avg_time_reached = avg_time_reached / n_reached
avg_trial_reached = avg_trial_reached / n_reached
avg_node_reached = avg_node_reached / n_reached
avg_total_time = total_time / (n_reached + not_reached)
avg_time_not_reached = avg_time_not_reached / not_reached

print "Averaged time reached:", avg_time_reached
print "Averaged trial reached:", avg_trial_reached
print "Averaged added nodes reached:", avg_node_reached
print "Averaged total time:", avg_total_time
print "Averaged time not reached:", avg_time_not_reached

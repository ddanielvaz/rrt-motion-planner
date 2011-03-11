# -*- coding: utf-8 -*-

fp = open("log.statistics", "r")

total_time = 0.0
not_reached = 0.0
n_reached = 0.0
avg_time_reached = 0.0
avg_trial_reached = 0.0
avg_node_reached = 0.0
un_avg_node_reached = 0.0
avg_time_not_reached = 0.0
avg_total_time = 0.0

count = 0.0

for line in fp:
    count += 1.0
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
        un_avg_node_reached += added_nodes

avg_time_reached = avg_time_reached / n_reached
avg_trial_reached = avg_trial_reached / n_reached
avg_node_reached = avg_node_reached / n_reached
avg_total_time = total_time / (n_reached + not_reached)
avg_time_not_reached = avg_time_not_reached / not_reached

un_avg_node_reached = un_avg_node_reached / (count - n_reached)

print "Total experiments:", count
print "Total time:", total_time
print "Goal reached %f times." % n_reached
print "Average of %f percent.", (n_reached/count)*100
print "Averaged time reached:", avg_time_reached
print "Averaged trial reached:", avg_trial_reached
print "Averaged added nodes reached:", avg_node_reached
print "Averaged total time:", avg_total_time
print "Averaged time not reached:", avg_time_not_reached
print "Unsuccessful Averaged added nodes reached:", un_avg_node_reached

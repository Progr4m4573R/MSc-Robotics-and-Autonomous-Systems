import numpy as np
#Computing averages for Tallon's performance across 10 runs for multiple scenarios

#Scenario 1
#speed of meanines spawning: 5
#number of bonuses: 3
#number of pits: 3
#partial visibility: TRUE

#GRID SIZE 15 by 15
Scenario_1_15_by_15_score = [4,4,8,7,3,6,6,7,14,6]
Scenario_1_15_by_15_simulation_clock = [8,9,17,14,6,13,12,14,29,12]
Scenario_1_15_by_15_mean_score = np.mean(Scenario_1_15_by_15_score)
Scenario_1_15_by_15_mean_survival_time = np.mean(Scenario_1_15_by_15_simulation_clock)

#GRID SIZE 10 by 10
Scenario_1_10_by_10_score = [2,12,28,13,30,11,5,14,22,9]
Scenario_1_10_by_10_simulation_clock = [4,4,16,7,21,3,10,8,5,18]
Scenario_1_10_by_10_mean_score = np.mean(Scenario_1_10_by_10_score)
Scenario_1_10_by_10_mean_survival_time = np.mean(Scenario_1_10_by_10_simulation_clock)

#GRID SIZE 5 by 5
Scenario_1_5_by_5_score = [11,0,23,12,0,3,2,0,1,1]
Scenario_1_5_by_5_simulation_clock = [2,1,6,4,1,6,4,1,2,3]
Scenario_1_5_by_5_mean_score = np.mean(Scenario_1_5_by_5_score)
Scenario_1_5_by_5_mean_survival_time = np.mean(Scenario_1_5_by_5_simulation_clock)

print("In Scenario 1 The mean score for a 15 by 15 grid was ",Scenario_1_15_by_15_mean_score)
print("In Scenario 1 The mean survival time for a 15 by 15 grid was ",Scenario_1_15_by_15_mean_survival_time)
print("In Scenario 1 The mean score for a 10 by 10 grid was ",Scenario_1_10_by_10_mean_score)
print("In Scenario 1 The mean survival time for a 10 by 10 grid was ",Scenario_1_10_by_10_mean_survival_time)
print("In Scenario 1 The mean score for a 5 by 5 grid was ",Scenario_1_5_by_5_mean_score)
print("In Scenario 1 The mean survival time for a 5 by 5 grid was ",Scenario_1_5_by_5_mean_survival_time)

#Scenario 2
#speed of meanines spawning: 10
#number of bonuses: 6
#number of pits: 6
#partial visibility: TRUE
#GRID SIZE 15 by 15
Scenario_2_15_by_15_score = [3,17,16,3,5,9,13,91,56,5]
Scenario_2_15_by_15_simulation_clock = [7,15,32,6,11,19,26,163,112,11]
Scenario_2_15_by_15_mean_score = np.mean(Scenario_2_15_by_15_score)
Scenario_2_15_by_15_mean_survival_time = np.mean(Scenario_2_15_by_15_simulation_clock)

#GRID SIZE 10 by 10
Scenario_2_10_by_10_score = [2,1,35,45,41,50,28,18,18,27]
Scenario_2_10_by_10_simulation_clock = [5,2,11,10,22,40,36,16,17,55]
Scenario_2_10_by_10_mean_score = np.mean(Scenario_2_10_by_10_score)
Scenario_2_10_by_10_mean_survival_time = np.mean(Scenario_2_10_by_10_simulation_clock)

#GRID SIZE 5 by 5
Scenario_2_5_by_5_score = [0,10,21,0,11,0,0,0,1,22]
Scenario_2_5_by_5_simulation_clock = [1,1,2,1,3,1,1,1,2,4]
Scenario_2_5_by_5_mean_score = np.mean(Scenario_2_5_by_5_score)
Scenario_2_5_by_5_mean_survival_time = np.mean(Scenario_2_5_by_5_simulation_clock)

print("In Scenario 2 The mean score for a 15 by 15 grid was ",Scenario_2_15_by_15_mean_score)
print("In Scenario 2 The mean survival time for a 15 by 15 grid was ",Scenario_2_15_by_15_mean_survival_time)
print("In Scenario 2 The mean score for a 10 by 10 grid was ",Scenario_2_10_by_10_mean_score)
print("In Scenario 2 The mean survival time for a 10 by 10 grid was ",Scenario_2_10_by_10_mean_survival_time)
print("In Scenario 2 The mean score for a 5 by 5 grid was ",Scenario_2_5_by_5_mean_score)
print("In Scenario 2 The mean survival time for a 5 by 5 grid was ",Scenario_2_5_by_5_mean_survival_time)

#Scenario 3
#speed of meanines spawning: 20
#number of bonuses: 12
#number of pits: 12
#partial visibility: TRUE

#GRID SIZE 15 by 15
Scenario_3_15_by_15_score = [12,1,39,16,1,22,7,2,46,58]
Scenario_3_15_by_15_simulation_clock = [5,2,59,12,2,25,14,5,53,97]
Scenario_3_15_by_15_mean_score = np.mean(Scenario_3_15_by_15_score)
Scenario_3_15_by_15_mean_survival_time = np.mean(Scenario_3_15_by_15_simulation_clock)

#GRID SIZE 10 by 10
Scenario_3_10_by_10_score = [11,22,52,1,2,24,8,32,2,4]
Scenario_3_10_by_10_simulation_clock = [3,5,65,2,5,28,16,5,5,9]
Scenario_3_10_by_10_mean_score = np.mean(Scenario_3_10_by_10_score)
Scenario_3_10_by_10_mean_survival_time = np.mean(Scenario_3_10_by_10_simulation_clock)

#GRID SIZE 5 by 5
#It was not possible to run the code with a 5 by 5 grid due to there being too many assets for the dimensions of the grid

print("In Scenario 3 The mean score for a 15 by 15 grid was ",Scenario_3_15_by_15_mean_score)
print("In Scenario 3 The mean survival time for a 15 by 15 grid was ",Scenario_3_15_by_15_mean_survival_time)
print("In Scenario 3 The mean score for a 10 by 10 grid was ",Scenario_3_10_by_10_mean_score)
print("In Scenario 3 The mean survival time for a 10 by 10 grid was ",Scenario_3_10_by_10_mean_survival_time)
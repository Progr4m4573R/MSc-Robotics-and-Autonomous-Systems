#
# Advanced Robotics, 2021-2022
# Paul Baxter
# Workshop Week 2
#
# code based on example: https://github.com/MJeremy2017/Reinforcement-Learning-Implementation/blob/master/GridWorld/gridWorld.py
# with description: https://towardsdatascience.com/reinforcement-learning-implement-grid-world-from-scratch-c5963765ebff
#
# Run with:     python q-learning.py
#
# May need following depedencies:
#   python -mpip install numpy
#
#
# Three classes in this file:
#  1. State: the board/maze
#  2. QAgent: a basic Q-learning agent
#  3. IRLAgent: a basic IRL agent, to be extended/modified
#
import matplotlib.pyplot as plt
import pandas as pd
from sre_constants import REPEAT
from statistics import mean
import numpy as np
from scipy import stats

np.random.seed(10)

# global variables
LOGGING = True         #set full logging to terminal or not...
EXPLORE = 0.3          #the explore proportion: (1-EXPLORE) for exloit
MANUAL_FEEDBACK = 0.1  #reward feedback from human: + and -
NEUTRAL_FEEDBACK = 0.05#if no feedback, this reward applied (+)
# maze states - leave alone for now
BOARD_ROWS = 3
BOARD_COLS = 4
WIN_STATE = (0, 3)
LOSE_STATE = (1, 3)
START = (2, 0)          #third row, first column

##########################################################
# The maze environment
##########################################################
class State:
    def __init__(self, state=START):
        self.board = np.zeros([BOARD_ROWS, BOARD_COLS])
        self.board[1, 1] = -1# negative reward for being in the pit
        self.state = state
        self.isEnd = False

    def giveReward(self):
        if self.state == WIN_STATE:
            return 1
        elif self.state == LOSE_STATE:
            return -1
        else:
            return 0

    def isEndFunc(self):
        if (self.state == WIN_STATE) or (self.state == LOSE_STATE):
            self.isEnd = True

    def nxtPosition(self, action):
        """
        action: up, down, left, right
        -------------
        0 | 1 | 2| 3|
        1 |
        2 |
        return next position
        """
        if action == "up":
            nxtState = (self.state[0] - 1, self.state[1])
        elif action == "down":
            nxtState = (self.state[0] + 1, self.state[1])
        elif action == "left":
            nxtState = (self.state[0], self.state[1] - 1)
        else:
            nxtState = (self.state[0], self.state[1] + 1)
        # if next state legal
        if (nxtState[0] >= 0) and (nxtState[0] <= 2):
            if (nxtState[1] >= 0) and (nxtState[1] <= 3):
                if nxtState != (1, 1):
                    return nxtState
        return self.state

    def showBoard(self):
        self.board[self.state] = 1
        for i in range(0, BOARD_ROWS):
            print('-----------------')
            out = '| '
            for j in range(0, BOARD_COLS):
                if self.board[i, j] == 1:
                    token = '*'
                if self.board[i, j] == -1:
                    token = 'X'
                if self.board[i, j] == 0:
                    token = '0'
                out += token + ' | '
            print(out)
        print('-----------------')


##########################################################
# Agent using basic Q-learning
##########################################################
class QAgent:

    def __init__(self):
        self.states = []
        self.numStates = []
        self.actions = ["up", "down", "left", "right"]
        self.State = State()
        self.lr = 0.2
        self.exp_rate = EXPLORE
        self.mean_moves = 0.0

        # initial state reward
        self.state_values = {}
        for i in range(BOARD_ROWS):
            for j in range(BOARD_COLS):
                self.state_values[(i, j)] = 0  # set initial value to 0

    def chooseAction(self):
        # choose action with most expected value
        mx_nxt_reward = 0
        action = ""

        if np.random.uniform(0, 1) <= self.exp_rate:
            action = np.random.choice(self.actions)
        else:
            # greedy action
            for a in self.actions:
                nxt_reward = self.state_values[self.State.nxtPosition(a)]
                if nxt_reward >= mx_nxt_reward:
                    action = a
                    mx_nxt_reward = nxt_reward
        return action

    def takeAction(self, action):
        position = self.State.nxtPosition(action)
        return State(state=position)

    def reset(self):
        self.states = []
        self.State = State()

    def play(self, rounds=10):
        i = 0
        print ("")
        print ("Q-LEARNING START")
        stepCounter = 0
        while i < rounds:
            # to the end of game back propagate reward
            if self.State.isEnd:
                # back propagate
                reward = self.State.giveReward()
                # explicitly assign end state to reward values
                self.state_values[self.State.state] = reward  # this is optional
                print ("--------------------------------------- Game End Reward", reward)
                print ("--------------------------------------- Num Steps Used: ", stepCounter)
                for s in reversed(self.states):
                    reward = self.state_values[s] + self.lr * (reward - self.state_values[s])
                    self.state_values[s] = round(reward, 3)
                self.reset()
                self.numStates.append(stepCounter)
                stepCounter = 0
                i += 1
            else:
                stepCounter += 1
                action = self.chooseAction()
                # append trace
                self.states.append(self.State.nxtPosition(action))
                if (LOGGING):
                    print("  current position {} action {}".format(self.State.state, action))
                # by taking the action, it reaches the next state
                self.State = self.takeAction(action)
                # mark is end
                self.State.isEndFunc()
                if (LOGGING):
                    print ("    |--> next state", self.State.state)

    def showValues(self):
        print ("")
        for i in range(0, BOARD_ROWS):
            print ("-------------------------------------")
            out = '| '
            for j in range(0, BOARD_COLS):
                out += str(self.state_values[(i, j)]).ljust(6) + ' | '
            print(out)
        print ("-------------------------------------")
        print ("Q-learning steps: ",self.numStates)
        Q_mean = np.mean(self.numStates)
        Q_max = np.max(self.numStates)
        Q_min = np.min(self.numStates)
        Q_median = np.median(self.numStates)
        Q_mode = stats.mode(self.numStates)
        Q_Standard_Deviation = np.std(self.numStates)
        print ("Q_mean: ", Q_mean,"\nQ_max: ", Q_max,"\nQ_min: ", Q_min,"\nQ_mode: ",Q_mode[0], "with",Q_mode[1],"\noccurences", "\nMedian:", Q_median,"\nStandard deviation:", Q_Standard_Deviation)
        print("")
        
        #------------plot QAgent states across runs as line graph-----------------
        #https://www.geeksforgeeks.org/plot-line-graph-from-numpy-array/

        output = np.array(self.numStates)
        print(type(output))
        y = self.numStates
        x = np.arange(0,len(self.numStates))
        plt.title("Line graph of Q-Learning Agent")
        plt.xlabel("number of runs")
        plt.ylabel("number of states taken per run")
        plt.plot(x, y, color ="red")
        plt.show()
        #------------------plot QAgent HISTOGRAM------------------------
        #https://www.w3schools.com/python/matplotlib_histograms.asp
        num_of_iterations = 40
        ret = np.random.normal(Q_mean, Q_Standard_Deviation, num_of_iterations)
        plt.hist(ret)
        plt.xlabel('run times')
        plt.ylabel('Number of actions taken to get to goal')
        plt.title('Plot of Q-Learnig Agent')
        plt.show()
        #------------Box plot QAgent Summary across runs -----------------
        QAgent_statistics_values = [Q_max, Q_min, Q_median]
        fig = plt.figure(figsize =(10, 7))
        plt.boxplot(QAgent_statistics_values)
        # show plot
        plt.title('Summary plot of Q-Learning Agent states')
        plt.show()



##########################################################################################################################################################################################
# Interactive RL agent
###########################################################################################################################################################################################
class IRLAgent:

    def __init__(self):
        self.states = []
        self.numStates = []
        self.actions = ["up", "down", "left", "right"]
        self.State = State()
        self.lr = 0.2
        self.exp_rate = EXPLORE
        self.bad_Actions = []
        # initial state reward
        self.state_values = {}
        for i in range(BOARD_ROWS):
            for j in range(BOARD_COLS):
                self.state_values[(i, j)] = 0  # set initial value to 0

    def chooseAction(self):
        # choose action with most expected value
        mx_nxt_reward = 0
        action = ""

        if np.random.uniform(0, 1) <= self.exp_rate:
            action = np.random.choice(self.actions)
        else:
            # greedy action
            for a in self.actions:
                nxt_reward = self.state_values[self.State.nxtPosition(a)]
                if nxt_reward >= mx_nxt_reward:
                    action = a
                    mx_nxt_reward = nxt_reward
        return action

    def takeAction(self, action):
        position = self.State.nxtPosition(action)
        return State(state=position)

    def reset(self):
        self.states = []
        self.State = State()

    def play(self, rounds=3):
        i = 0
        print ("")
        print ("IRL START")
        print ("")
        stepCounter = 0
        while i < rounds :
            # to the end of game back propagate reward
            if self.State.isEnd:
                # back propagate
                reward = self.State.giveReward()
                # explicitly assign end state to reward values
                self.state_values[self.State.state] = reward  # this is optional
                print ("--------------------------------------- Game End Reward:", reward)
                print ("--------------------------------------- Num Steps Used: ", stepCounter)
                for s in reversed(self.states):
                    reward = self.state_values[s] + self.lr * (reward - self.state_values[s])
                    self.state_values[s] = round(reward, 3)
                self.reset()
                self.numStates.append(stepCounter)
                stepCounter = 0
                i += 1
            else:
                print("_________________________________________________")
                stepCounter += 1
                self.State.showBoard()
                action = self.chooseAction()
                if action not  in self.bad_Actions:
                    action = action
                    self.bad_Actions.clear()
                    # # append trace
                    # self.states.append(self.State.nxtPosition(action))
                    current_state = self.State.state    #current state before action is executed
                    if (LOGGING):
                        print("  current position {} action {}".format(self.State.state, action))
                    # by taking the action, it reaches the next state
                    #self.State = self.takeAction(action)
                    # mark is end
                    self.State.isEndFunc()
                    if (LOGGING):
                        print ("    |--> next state", self.State.state)
                    # for IRL allow user to define reward:
                    #  - get reward from user:
                    feedback = input("      *was* this g(ood) or b(ad): ")
                    u_reward = 0
                    if feedback == "g":
                        u_reward = MANUAL_FEEDBACK
                        #reward the state and then repeat the action if the action lead to that state

                        # append trace
                        self.states.append(self.State.nxtPosition(action))
                        current_state = self.State.state    #current state before action is executed
                        if (LOGGING):
                            print("  current position {} action {}".format(self.State.state, action))
                        # by taking the action, it reaches the next state
                        self.State = self.takeAction(action)
                        # mark is end
                        self.State.isEndFunc()
                        if (LOGGING):
                            print ("    |--> next state", self.State.state)

                    elif feedback == "b":
                        u_reward = -MANUAL_FEEDBACK
                        # if the suggested action is the same as previous state then stay
                        self.State.state == current_state     #current state before action is executed
                        self.bad_Actions.append(action)
                        if (LOGGING):
                            print("  current position {} action {}".format(self.State.state, action))
                        self.State.isEndFunc()
                        if (LOGGING):
                            print ("    |--> bad action, staying put", self.State.state)
                    else:
                        #not recognised, assume ok...
                        u_reward = NEUTRAL_FEEDBACK
                    #  - update the value of the current state only but should reward the action given the state
                    reward = self.state_values[current_state] + self.lr * (u_reward - self.state_values[current_state])


                    self.state_values[current_state] = round(reward, 3)
               
    def showValues(self):
        for i in range(0, BOARD_ROWS):
            print ("-------------------------------------")
            out = '| '
            for j in range(0, BOARD_COLS):
                out += str(self.state_values[(i, j)]).ljust(6) + ' | '
            print(out)
        print ("-------------------------------------")
        print (self.numStates)
        #UNCOMMENT for summary statistics
        IRL_mean = np.mean(self.numStates)
        IRL_max = np.max(self.numStates)
        IRL_min = np.min(self.numStates)
        IRL_median = np.median(self.numStates)
        IRL_mode = stats.mode(self.numStates)
        IRL_standard_deviation = np.mean(self.numStates)
        print ("IRL_mean: ", IRL_mean,"\nIRL_max: ", IRL_max,"\nIRL_min: ", IRL_min,"\nIRL_mode: ",IRL_mode[0], "with",IRL_mode[1],"occurences","\nMedian:", IRL_median, "\nStandard deviation:", IRL_standard_deviation)
        print ("")

        #------------plot IRL states across runs as line graph-----------------
        #https://www.geeksforgeeks.org/plot-line-graph-from-numpy-array/
        np.array(self.numStates)
        y = self.numStates
        x = np.arange(0,len(self.numStates))
        plt.title("Line graph of IRL Agent")
        plt.xlabel("number of runs")
        plt.ylabel("number of states taken to get to goal")
        plt.plot(x, y, color ="red")
        plt.show()
        #------------------plot IRL HISTOGRAM------------------------
        #https://www.w3schools.com/python/matplotlib_histograms.asp
        num_of_iterations = 40
        ret = np.random.normal(IRL_mean, IRL_standard_deviation, num_of_iterations)
        plt.hist(ret)
        plt.xlabel('run times')
        plt.ylabel('Number of actions taken to get to goal')
        plt.title('Histogram plot of IRL Agent')
        plt.show()
        #------------Box plot QAgent Summary across runs -----------------
        IRL_statistics_values = [IRL_max, IRL_min, IRL_median]
        fig = plt.figure(figsize =(10, 7))
        plt.boxplot(IRL_statistics_values)
        # show plot
        plt.title('Summary plot of IRL Agent states')
        plt.show()


##########################################################
# Main
##########################################################
if __name__ == "__main__":
    ag = QAgent()
    ag.play(40)

    irl = IRLAgent()
    irl.play(40)            # <--- Uncomment this to enable the IRLAgent

    print ("_________________________________________________")
    # print ("")
    # print ("Q-learning agent:")
    # print(ag.showValues())
    
    print ("")
    print ("IRL agent:")
    print(irl.showValues())
    print ("")

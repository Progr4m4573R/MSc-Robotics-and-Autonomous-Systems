#
# Advanced Robotics, 2021-2022
# Paul Baxter
# Workshop Week 3
#
# code based on example: https://github.com/MJeremy2017/Reinforcement-Learning-Implementation/blob/master/GridWorld/gridWorld.py
#
# Run with:     python3 lecture3-simulation.py
#
# May need following depedencies:
#   python3 -mpip install numpy
#
# Three classes in this file:
#  1. State: the board/maze
#  2. Agent: a basic Q-learning agent (value iteration)
#  3. IRLAgentPlus: a different basic IRL agent, to be extended/modified
#


import numpy as np

# global variables
# simulation parameters
N_QAGENT = 50           #iterations of the basic value iteration agent
N_IRLAGENT = 1          #number of iterations of IRLplus to perform
EXPLORE = 0.3           #the explore proportion: (1-EXPLORE) for exloit
MANUAL_FEEDBACK = 0.1   #reward feedback from human: + and -
NEUTRAL_FEEDBACK = 0.05 #if no feedback, this reward applied (+)
LOGGING = False         #set full logging to terminal or not...
# maze setup - leave alone for now
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
        self.board[1, 1] = -1
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
                    token = 'o'
                if self.board[i, j] == -1:
                    token = 'X'
                if self.board[i, j] == 0:
                    token = ' '
                out += token + ' | '
            print(out)
        print('-----------------')


##########################################################
# Agent using basic value iteration
##########################################################
class Agent:

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
        print ("")
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
        print (self.numStates)
        print ("")


##########################################################
# Interactive RL agent - changed from week 2
##########################################################
class IRLAgentPlus:

    def __init__(self):
        self.states = []
        self.numStates = []
        self.actions = ["up", "down", "left", "right"]
        self.State = State()
        self.lr = 0.2
        self.exp_rate = EXPLORE

        # initial state reward
        self.state_values = {}
        for i in range(BOARD_ROWS):
            for j in range(BOARD_COLS):
                self.state_values[(i, j)] = 0  # set initial value to 0

    def chooseAction(self, rand=False, actAvoid=0):
        # choose action with most expected value,
        #  unless we want an explicitly different action...
        mx_nxt_reward = 0
        action = ""

        if rand == True:    #this part intentionally left uncommented: what does it do?
            flag = True
            while (flag):
                action = np.random.choice(self.actions)
                if action == actAvoid:
                    flag = True
                else:
                    flag = False
            return action

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
        print ("IRLplus START")
        print ("")
        stepCounter = 0
        while i < rounds:
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
                print("Next action chosen: ", action)
                # for IRLplus, allow user to judge the proposed action:
                #  - get feedback from user:
                feedback = input("      *will* this action be g(ood) or b(ad): ")    #if using python2, may need to replace "input" with "raw_input"
                u_reward = 0
                if feedback == "g":
                    action = action
                elif feedback == "b":
                    #choose a new action, not the same as existing...
                    newAction = 0
                    newAction = self.chooseAction(True, action)
                    action = newAction
                else:
                    #not recognised, assume ok, so carry on...
                    action = action
                # append trace
                self.states.append(self.State.nxtPosition(action))
                current_state = self.State.state    #current state before action is executed
                # by taking the action, it reaches the next state
                self.State = self.takeAction(action)
                # mark is end
                self.State.isEndFunc()
                self.State.showBoard()
                #  - get reward from user:
                print("      action actually used: ", action)
                feedback = input("      /was/ this action g(ood) or b(ad): ")
                u_reward = 0
                if feedback == "g":
                    u_reward = MANUAL_FEEDBACK
                    # keep the same action as proposed
                elif feedback == "b":
                    u_reward = -MANUAL_FEEDBACK
                else:
                    #not recognised, assume ok, so carry on...
                    u_reward = NEUTRAL_FEEDBACK
                #  - update the value of the current state only
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
        print ("")


##########################################################
# Main
##########################################################
if __name__ == "__main__":

    ag = Agent()
    ag.play(N_QAGENT)

    irlp = IRLAgentPlus()
    #irlp.play(N_IRLAGENT)            # <--- Uncomment this to enable IRLAgentPlus

    print ("_________________________________________________")
    print ("")
    print ("Q-learning agent: ", N_QAGENT, " iterations")
    print(ag.showValues())
    print ("")
    print ("IRLplus agent: ", N_IRLAGENT, " iterations")
    print(irlp.showValues())

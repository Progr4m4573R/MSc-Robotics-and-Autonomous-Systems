import numpy as np
import random
import time
import math
import gym

def discretiseState(state):
  discretized = [round(state[0],1), round(state[1],1), round(state[2],1), round(state[3],1)]
  return discretized	

def eGreedy(s, Q, A, epsilon):
  if random.random()<epsilon:
    rand_index = random.randint(0,len(A)-1)
    return rand_index
  else:
    action_values = []
    action_values.append(getQValue(s, 0, Q))
    action_values.append(getQValue(s, 1, Q))
    best_a = np.argmax(action_values)
    return best_a
    
def getQValue(s, a, Q):
  try:
    action_values = Q[str(s)]
    return action_values[a]
  except:
    return 0

def updateQTable(s, a, Q, new_qvalue):
  try:
    action_values = Q[str(s)]
    if a==0: new_action_values = [new_qvalue, action_values[1]]
    else: new_action_values = [action_values[0], new_qvalue]
    Q[str(s)] = new_action_values
  except: 
    if a==0: action_values = [new_qvalue, 0]
    else: action_values = [0, new_qvalue]
    Q[str(s)] = action_values
  return Q

env = gym.make('CartPole-v0')    
env.seed(42)
 
A = ['left','right']
Q = {}
episodes = 10000
epsilon = 1
epsilon_min = 0.05
learning_rate = 0.1
discount_rate = 0.99
window_size = 100
total_steps = 0
total_reward = 0
solved = False

for i in range(0,episodes+1000000):
  t = 0
  s = env.reset()
  s = discretiseState(s)
  
  while True:
    if solved: env.render()
    a = eGreedy(s, Q, A, epsilon)
    s_prime, r, done, info = env.step(a)
    s_prime = discretiseState(s_prime)
    r = 1-(1/t+1) if done else r
    
    current_qvalue = getQValue(s, a, Q)
    a_prime = eGreedy(s_prime, Q, A, 0)
    next_qvalue = getQValue(s_prime, a_prime, Q)
    new_qvalue = current_qvalue + learning_rate * (r+discount_rate*next_qvalue-current_qvalue)
    Q = updateQTable(s, a, Q, new_qvalue)
    
    epsilon = 1000/(1000+i)
    total_steps += 1
    total_reward += r
    s = s_prime
    t = t+1
    if done: break

  if i>0 and i%window_size==0: 
    avg_steps = float(total_steps/window_size)
    avg_reward = float(total_reward/window_size)
    print("avg_steps="+str(avg_steps)+" avg_reward="+str(avg_reward)+" epsilon="+str(epsilon)+" episode="+str(i))
    total_steps = 0
    total_reward = 0   

  if epsilon<=epsilon_min and solved == False:
    print("Solved at episode {}!".format(i))
    solved = True

env.close()

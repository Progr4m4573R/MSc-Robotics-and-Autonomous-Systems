import numpy as np
import random
import math
import matplotlib.pyplot as plt

class SimpleAgent():
    GridWorld = np.zeros((10,10))
    A = ['up','down','left','right']
    Q = np.zeros((len(A),len(GridWorld),len(GridWorld)))
    episodes = 3000
    epsilon = 1
    learning_rate = 0.1
    discount_rate = 0.99
    goal_state = [5,5]
    window_size = 50
    total_steps = 0
    total_reward = 0
    logged_data = []

    def __init__(self): 
        for j in range(1,4): self.GridWorld[2][j] = float("NaN")
        for j in range(6,9): self.GridWorld[2][j] = float("NaN")
        for i in range(2,8): self.GridWorld[i][4] = float("NaN")
        self.GridWorld[3][3] = -1
        self.GridWorld[4][5] = -1
        self.GridWorld[4][6] = -1
        self.GridWorld[5][5] = +1
        self.GridWorld[5][6] = -1
        self.GridWorld[5][8] = -1
        self.GridWorld[6][8] = -1
        self.GridWorld[7][3] = -1
        self.GridWorld[7][5] = -1
        self.GridWorld[7][6] = -1

    def eGreedy(self, s, randomness):
        if random.random()<randomness:
            rand_index = random.randint(0,len(self.A)-1)
            return rand_index
        else:
            candidates = []
            for i in range(0,len(self.A)):
                candidates.append(self.Q[i][s[0]][s[1]])
            best_a = np.argmax(candidates)
            return best_a

    def getNextState(self, s, a):
        if a=='up': s_prime = [s[0]-1,s[1]]
        elif a=='down': s_prime = [s[0]+1,s[1]]
        elif a=='left': s_prime = [s[0],s[1]-1]
        elif a=='right': s_prime = [s[0],s[1]+1]
        if s_prime[0]<0 or s_prime[0]>=len(self.GridWorld) or \
           s_prime[1]<0 or s_prime[1]>=len(self.GridWorld): s_prime = s

        value = self.GridWorld[s_prime[0]][s_prime[1]]
        if math.isnan(value): s_prime = s
        return s_prime
        
    def logPerformance(self, episode):
        avg_steps = float(self.total_steps/self.window_size)
        avg_reward = float(self.total_reward/self.window_size)
        self.logged_data.append([episode, avg_steps, avg_reward])
        print("avg_steps="+str(avg_steps)+" avg_reward="+\
              str(avg_reward)+" epsilon="+str(self.epsilon)+" i="+str(episode))	
    
    def plotPerformance(self):
        self.logged_data = np.array(self.logged_data)
        fig, axs = plt.subplots(2)
        fig.suptitle('Learning Curves')
        axs[0].set(xlabel='Episode', ylabel='Avg. Timesteps')
        axs[1].set(xlabel='Episode', ylabel='Avg. Reward')
        axs[0].set_yscale('log')
        axs[1].set_yscale('log')
        axs[0].plot(self.logged_data[:,0], self.logged_data[:,1], 'tab:green')
        axs[1].plot(self.logged_data[:,0], self.logged_data[:,2], 'tab:red')
        plt.show()
         
    def trainAgent(self):
        for i in range(0,self.episodes+1):
            t = 0 
            s = [0,0]
      
            while True:
                a = self.eGreedy(s, self.epsilon)
                s_prime = self.getNextState(s, self.A[a])
                r = self.GridWorld[s_prime[0]][s_prime[1]]
    
                current_qvalue = self.Q[a][s[0]][s[1]]
                a_prime = self.eGreedy(s_prime, 0)
                next_qvalue = self.Q[a_prime][s_prime[0]][s_prime[1]]
                new_qvalue = current_qvalue+self.learning_rate*(r+self.discount_rate*next_qvalue-current_qvalue)
                self.Q[a][s[0]][s[1]] = new_qvalue

                self.epsilon = 100/(100+i)
                self.total_steps += 1
                self.total_reward += r
                s = s_prime
                t = t+1
                if s == self.goal_state: break
        
            if i>0 and i%self.window_size==0: 
                self.logPerformance(i)
                self.total_steps = 0
                self.total_reward = 0   
  
        print(str(self.GridWorld)+'\n'+str(self.Q))
        
sa = SimpleAgent()
sa.trainAgent()
sa.plotPerformance()

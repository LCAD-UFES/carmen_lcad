
import sys
import numpy as np
from collections import deque
from rl.util import relative_pose


class ReplayBuffer:
    def __init__(self, capacity, n_trad, n_her, max_episode_size):
        """
        :param capacity: max size of the replay memory
        :param n_trad: number of traditional transitions per batch
        :param n_her: number of her transitions per batch
        :param: max_episode_size to compute the reward in goals chosen in hindsight
        """
        print('ReplayBuffer with n_trad={} and n_her={}'.format(n_trad, n_her))
        self.capacity = capacity
        self.n_trad = n_trad
        self.n_her = n_her
        self.max_episode_size = max_episode_size
        self.stack = deque()

    def add(self, episodes):
        for e in episodes:
            if len(e) > 1:
                self.stack.append(e)
                if len(self.stack) > self.capacity:
                    self.stack.popleft()

    def sample(self):
        batch = {'laser': [], 'state': [], 'goal': [], 'rew': [], 'act': [],
                 'next_laser': [], 'next_state': [], 'next_goal': [], 'is_final': []}

        # sample a set of episodes
        n = len(self.stack)
        e_ids = np.random.randint(0, n, self.n_trad)

        # add a set of transitions to the batch
        transitions_not_final = []
        for i in range(self.n_trad):
            e = e_ids[i]
            t = np.random.randint(0, len(self.stack[e]))
            goal = relative_pose(self.stack[e][t][0]['pose'], self.stack[e][t][3]) + [self.stack[e][t][3][3]]
            
            batch['laser'].append(self.stack[e][t][0]['laser'])
            batch['state'].append([self.stack[e][t][0]['pose'][3]])
            batch['act'].append(self.stack[e][t][1])
            batch['rew'].append([self.stack[e][t][2]])
            batch['goal'].append(goal)

            # check if the transition is the last one in the episode
            if t < len(self.stack[e]) - 1:
                transitions_not_final.append([e, t])
                next_goal = relative_pose(self.stack[e][t + 1][0]['pose'], self.stack[e][t][3]) + [self.stack[e][t][3][3]]
                batch['next_laser'].append(self.stack[e][t + 1][0]['laser'])
                batch['next_state'].append([self.stack[e][t + 1][0]['pose'][3]])
                batch['next_goal'].append(next_goal)
                batch['is_final'].append([0.0])
            else:
                # If the transition is the last one, we don't have a next observation. In this case, we copy the
                # current observation as next one. This value will be ignored when computing the target q.
                batch['next_laser'].append(self.stack[e][t][0]['laser'])
                batch['next_state'].append([self.stack[e][t][0]['pose'][3]])
                batch['next_goal'].append(goal)
                batch['is_final'].append([1.0])

        # generate additional training data using her
        """
        Os caras da OpenAI usaram transicoes dos mesmos episodios acima para escolher novos goals. 
        Eh errado escolher transicoes de outros episodios?
        """
        # if len(transitions_not_final) > 0:
        for i in range(self.n_her):
            # e, t = transitions_not_final[np.random.randint(len(transitions_not_final))]
            e = np.random.randint(0, n)
            t = np.random.randint(0, len(self.stack[e]) - 1)

            future_t = np.random.randint(t + 1, len(self.stack[e]))
            her_episode_size = future_t - t
            her_return = (self.max_episode_size - her_episode_size) / (self.max_episode_size - 1)
            rew = (her_return / her_episode_size)
            is_final = 0.0

            """
            if her_episode_size <= 1:
                # her_return = (self.max_episode_size + 1 - her_episode_size) / self.max_episode_size
                # rew = (her_return / her_episode_size)
                rew = 1.0
                is_final = 1.0
            else:
                rew = 0.
                is_final = 0.0
            """

            # rew = 1. / self.max_episode_size

            batch['laser'].append(self.stack[e][t][0]['laser'])
            batch['state'].append([self.stack[e][t][0]['pose'][3]])
            batch['act'].append(self.stack[e][t][1])
            batch['rew'].append([rew])

            her_goal = self.stack[e][future_t][0]['pose'][:4]
            goal = relative_pose(self.stack[e][t][0]['pose'], her_goal) + [her_goal[3]]
            next_goal = relative_pose(self.stack[e][t + 1][0]['pose'], her_goal) + [her_goal[3]]

            batch['goal'].append(goal)
            batch['next_laser'].append(self.stack[e][t + 1][0]['laser'])
            batch['next_state'].append([self.stack[e][t + 1][0]['pose'][3]])
            batch['next_goal'].append(next_goal)
            batch['is_final'].append([is_final])

            """
            print("pose", self.stack[e][t][0]['pose'], "\n"
                  "goal", self.stack[e][future_t][0]['pose'][:4],
                  "rew", rew, "dt:", her_episode_size)
            """

        #import pprint
        #pprint.pprint(batch)

        return batch




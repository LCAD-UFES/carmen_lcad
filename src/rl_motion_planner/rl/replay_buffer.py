
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
        self.capacity = capacity
        self.n_trad = n_trad
        self.n_her = n_her
        self.max_episode_size = max_episode_size
        self.stack = deque()

    def add(self, episodes):
        for e in episodes:
            self.stack.append(e)
            if len(self.stack) > self.capacity:
                self.stack.popleft()

    def sample(self):
        batch = {'laser': [], 'state': [], 'goal': [], 'rew': [], 'act': [],
                 'next_laser': [], 'next_state': [], 'is_final': []}

        # sample a set of episodes
        n = len(self.stack)
        e_ids = np.random.randint(0, n, self.n_trad)
        t_ids = []

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

            t_ids.append(t)

            # check if the transition is the last one in the episode
            if t < len(self.stack[e]) - 1:
                transitions_not_final.append([e, t])
                batch['next_laser'].append(self.stack[e][t + 1][0]['laser'])
                batch['next_state'].append([self.stack[e][t + 1][0]['pose'][3]])
                batch['is_final'].append([0.0])
            else:
                # If the transition is the last one, we don't have a next observation. In this case, we copy the
                # current observation as next one. This value will be ignored when computing the target q.
                batch['next_laser'].append(self.stack[e][t][0]['laser'])
                batch['next_state'].append([self.stack[e][t][0]['pose'][3]])
                batch['is_final'].append([1.0])

        # generate additional training data using her
        if len(transitions_not_final) > 0:
            """
            Os caras da OpenAI usaram transicoes dos mesmos episodios acima para escolher novos goals. 
            Eh errado escolher transicoes de outros episodios?
            """
            for _ in range(self.n_her):
                e, t = transitions_not_final[np.random.randint(len(transitions_not_final))]

                future_t = np.random.randint(t + 1, len(self.stack[e]))
                her_episode_size = future_t - t
                rew = (self.max_episode_size - her_episode_size) / (self.max_episode_size * her_episode_size)

                batch['laser'].append(self.stack[e][t][0]['laser'])
                batch['state'].append([self.stack[e][t][0]['pose'][3]])
                batch['act'].append(self.stack[e][t][1])
                batch['rew'].append([rew])

                her_goal = self.stack[e][future_t][0]['pose'][:4]
                goal = relative_pose(self.stack[e][t][0]['pose'], her_goal) + [her_goal[3]]

                batch['goal'].append(goal)
                batch['next_laser'].append(self.stack[e][t + 1][0]['laser'])
                batch['next_state'].append([self.stack[e][t + 1][0]['pose'][3]])
                batch['is_final'].append([0.0])

        return batch





import numpy as np
from collections import deque


# TODO: definir essa funcao junto com as coisas do carmen, e importar ela de la'.
def estimate_reward_for_new_goal(episode, current_transition, goal_transition):
    return 1.0


class ReplayBuffer:
    def __init__(self, capacity, n_trad, n_her):
        """
        :param capacity: max size of the replay memory
        :param n_trad: number of traditional transitions per batch
        :param n_her: number of her transitions per batch
        """
        self.capacity = capacity
        self.n_trad = n_trad
        self.n_her = n_her
        self.stack = deque()

    def add(self, episodes):
        for e in episodes:
            self.stack.append(e)
            if len(self.stack) > self.capacity:
                self.stack.pop(0)

    def sample(self):
        batch = {'obs': [], 'goal': [], 'rew': [], 'act': []}

        # sample a set of episodes
        n = len(self.stack)
        e_ids = np.random.randint(0, n, self.n_trad)
        t_ids = []

        # add a set of transitions to the batch
        transitions_not_final = []
        for i in range(self.n_trad):
            e = e_ids[i]
            t = np.random.randint(0, len(self.stack[e]))
            batch['obs'].append(self.stack[e][t][0])
            batch['act'].append(self.stack[e][t][1])
            batch['rew'].append(self.stack[e][t][2])
            batch['goal'].append(self.stack[e][t][3])
            t_ids.append(t)

            if t < len(self.stack[e]) - 1:
                transitions_not_final.append([e, t])

        # generate additional training data using her
        if len(transitions_not_final) > 0:
            """
            Os caras da OpenAI usaram transicoes dos mesmos episodios acima para escolher novos goals. 
            Eh errado escolher transicoes de outros episodios?
            """
            for _ in range(n_her):
                e, t = np.random.choice(transitions_not_final)
                future_t = np.random.randint(0, len(self.stack[e]) - t)
                rew = estimate_reward_for_new_goal(self.stack[e], t, future_t)
                batch['goal'].append(self.stack[e][future_t][3])
                batch['obs'].append(self.stack[e][t][0])
                batch['act'].append(self.stack[e][t][1])
                batch['rew'].append(rew)

        return batch




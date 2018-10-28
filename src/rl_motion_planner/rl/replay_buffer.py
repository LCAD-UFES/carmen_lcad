

import sys
import numpy as np
from rl.util import relative_pose, dist


class ReplayBuffer:
    def __init__(self, params, reward_generator):
        self.capacity = params['replay_memory_capacity']
        self.n_trad = params['batch_size'] * (1. - params['her_rate'])
        self.batch_size = params['batch_size']
        self.rgen = reward_generator
        self.buffer = []
        self.batch = dict()


    def __len__(self):
        return len(self.buffer)


    def add(self, episodes):
        for e in episodes:
            self.buffer.append(e)
            if len(self.buffer) > self.capacity:
                self.buffer.pop(0)


    def get_batch(self):
        self._reset_batch()

        for i in range(self.batch_size):
            if i < self.n_trad:
                self._sample_and_add_transition_to_batch(is_her=False)
            else:
                self._sample_and_add_transition_to_batch(is_her=True)

        return self.batch


    def _reset_batch(self):
        self.batch['agent_states'] = []
        self.batch['lasers'] = []
        self.batch['goals'] = []
        self.batch['rewards'] = []
        self.batch['cmds'] = []
        self.batch['is_finals'] = []
        self.batch['next_agent_states'] = []
        self.batch['next_lasers'] = []
        self.batch['next_goals'] = []


    def _sample_and_add_transition_to_batch(self, is_her=False):
        # sample an episode
        episode = self.buffer[np.random.randint(len(self.buffer))]
        transitions = episode['transitions']
        
        n = len(transitions)
        
        # If we are using her we must have at least 
        # one posterior observation to use as fake goal.
        limit = n
        if is_her: 
            limit = n-1
        
        # sample a transition
        t_id = np.random.randint(limit)
        t = episode['transitions'][t_id]

        # obtain the transition data
        obs, cmd = t[0], t[1]
        rew, goal, is_final = self._compute_goal_reward_and_final_flag(is_her, t, episode, t_id, n, obs)
        
        if is_final: 
            next_obs = transitions[t_id + 1][0]
        else:
            # This value is dummy because Q_next is not used when is_finals is 1.0.
            next_obs = obs

        # add to batch
        self._add_to_batch(obs, cmd, next_obs, rew, goal, is_final)


    def _pose_to_goal(self, pose):
        goal = np.zeros(4)
        goal[:3] = pose[:3]
        return goal


    def _compute_goal_reward_and_final_flag(self, is_her, t, episode, t_id, n, obs):
        # Traditional samples 
        if not is_her:
            rew = t[2]
            goal = episode['goal']
            is_final = 1.0 if (t_id >= n - 1) else 0.
        # HER samples 
        else:
            assert (t_id < n - 1)
            # sample an observation from an episode to be a fake goal.
            # the goals are assumed to have same format as poses.
            goal_id = np.random.randint(t_id + 1, n)
            transitions = episode['transitions']
            goal = self._pose_to_goal(transitions[goal_id][0]['pose'])
            
            # The fake episode would have ended as soon as the agent reaches the goal.
            her_episode_size = goal_id
            
            is_final = 1.0 if ((goal_id - t_id) <= 1) else 0.0
            is_final_flag = True if is_final == 1.0 else 0.0 
            
            rew = self.rgen.reward(her_episode_size, obs, 
                                   goal=goal, 
                                   achieved=True, 
                                   is_final=is_final_flag)
        
        return rew, goal, is_final


    def _state_from_obs(self, obs):
        return obs['pose'][3:]


    def _add_to_batch(self, obs, cmd, next_obs, rew, goal, is_final):
        g_pose = relative_pose(obs['pose'], goal)
        g_next = relative_pose(next_obs['pose'], goal)
        
        s = self._state_from_obs(obs)
        n = self._state_from_obs(next_obs)
        
        self.batch['agent_states'].append(s)
        self.batch['lasers'].append(obs['laser'])
        self.batch['goals'].append(g_pose)
        self.batch['rewards'].append([rew])
        self.batch['cmds'].append(cmd)
        self.batch['is_finals'].append([is_final])
        self.batch['next_agent_states'].append(n)
        self.batch['next_lasers'].append(next_obs['laser'])
        self.batch['next_goals'].append(g_next)



            

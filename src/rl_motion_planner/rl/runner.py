
import time


def rollout(env, policy=None):
    episode = {'states': [], 'actions': [], 'rewards': []}

    init = time.time()
    done = False
    ret = 0

    state = env.reset()
    
    while not done:
        if policy is None:
            action = env.action_space.sample()
        else:
            action = policy.forward(state)
        
        new_state, rw, done, _ = env.step(action)
        
        episode['states'].append(state)
        episode['actions'].append(action)
        episode['rewards'].append(rw)

        state = new_state
        ret += rw

    episode['return'] = ret
    episode['info'] = {'duration': time.time() - init}
    
    return episode

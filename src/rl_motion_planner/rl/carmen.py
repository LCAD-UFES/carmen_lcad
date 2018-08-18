
import carmen_comm.carmen_comm as carmen_comm


class CarmenEnv:
    def __init__(self, rddf):
        carmen_path = os.environ['CARMEN_HOME']
        rddf_path = carmen_path + '/data/rndf/' + rddf
        self.rddf = [[float(field) for field in line.rstrip().rsplit(' ')]
                     for line in open(rddf_path, 'r').readlines()]
        carmen_comm.init()

    def reset(self):
        obs = None
        goal = None

        init_pos_id = np.random.randint(30, len(self.rddf) - 30)
        init_pos = rddf[init_pos_id]
        carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])

        forw_or_back = np.random.randint(2) * 2 - 1
        goal_id = init_pos_id + np.random.randint(10, 30) * forw_or_back
        goal = rddf[goal_id]



        return obs, goal

    def step(self, cmd):
        obs = None
        rw = None
        success = None
        hit_obstacle = None
        starved = None

        done = success or hit_obstacle or starved
        info = {'success': success, 'hit_obstacle': hit_obstacle, 'starved': starved}

        return obs, rw, done, info

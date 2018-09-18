
import time
import os
from rl.util import dist, ackerman_motion_model, draw_rectangle
import numpy as np
import cv2
import carmen_comm.carmen_comm as carmen


class SimpleEnv:
    def __init__(self, params):
        self.params = params

        if self.params['model'] == 'ackerman':
            self.env_size = 100.0
            self.zoom = 3.5
            self.max_speed = 10.0
            self.wheel_angle = np.deg2rad(28.)
            self.goal_radius = 2.0
            self.dt = 0.2
        else:
            self.env_size = 9.0
            self.zoom = 20.0
            self.goal_radius = 0.5
            self.dt = 1.0

        # 100 rays to support convolutional laser preprocessing
        self.laser = np.zeros(100).reshape(100, 1)

    def reset(self):
        self.pose = self.previous_p = np.zeros(4)

        self.env_border = int(self.env_size - 0.1 * self.env_size)
        self.goal  = np.array(list((np.random.random(2) * 2.0 - 1.0) * self.env_border) + [0., 0.])
        self.obstacles = []
        self.n_steps = 0

        return {'pose': np.copy(self.pose), 'laser': self.laser}, self.goal

    def step(self, cmd):
        self.n_steps += 1
        self.previous_p = np.copy(self.pose)

        if self.params['model'] == 'ackerman':
            v = cmd[0] * self.max_speed
            phi = cmd[1] * self.wheel_angle
            # v = 5.
            # phi = cmd[0] * self.wheel_angle
            self.pose = ackerman_motion_model(self.pose, v, phi, dt=self.dt)
        else:
            self.pose[0] += cmd[0] * self.dt
            self.pose[1] += cmd[1] * self.dt

        self.pose = np.clip(self.pose, -self.env_border, self.env_border)

        success = True if dist(self.pose, self.goal) < self.goal_radius else False
        starved = True if self.n_steps >= self.params['n_steps_episode'] else False

        info = {'success': success, 'hit_obstacle': False, 'starved': starved}
        done = success or starved

        return {'pose': np.copy(self.pose), 'laser': self.laser}, done, info

    def finalize(self):
        pass

    def line_params(self, line_vs):
        ms = [(v1[1] - v2[1]) / (v1[0] - v2[0]) for v1, v2 in line_vs]
        bs = [line_vs[i][0][1] - ms[i] * line_vs[i][0][0] for i in range(len(ms))]

        return list(zip(ms, bs))

    def view(self):
        goal = self.goal
        ob = self.pose

        img_size = int(self.zoom * self.env_size * 2)
        img = np.ones((img_size, img_size, 3))

        p1 = (int(img.shape[1]/2), 0)
        p2 = (int(img.shape[1]/2), img.shape[0])
        cv2.line(img, p1, p2, (1, 0, 0))

        p1 = (0, int(img.shape[0]/2))
        p2 = (img.shape[1], int(img.shape[0]/2))
        cv2.line(img, p1, p2, (1, 0, 0))

        g_px = int(goal[0] * self.zoom + img.shape[1] / 2)
        g_py = int(goal[1] * self.zoom + img.shape[0] / 2)

        img = cv2.circle(img, (g_px, g_py), int(self.goal_radius * self.zoom), (0, 1, 0), -1)

        for o in self.obstacles:
            img = o.draw(img, self.zoom)

        x = int(ob[0] * self.zoom + img.shape[1] / 2)
        y = int(ob[1] * self.zoom + img.shape[0] / 2)

        cv2.circle(img, (x, y), 2, (0, 0, 1), 0)

        x1 = int(self.previous_p[0] * self.zoom + img.shape[1] / 2)
        y1 = int(self.previous_p[1] * self.zoom + img.shape[0] / 2)

        cv2.circle(img, (x1, y1), 2, (0, 0, 1), 0)
        cv2.line(img, (x, y), (x1, y1), (0, 0, 1 ), 1)

        if self.params['model'] == 'ackerman':
            draw_rectangle(img, self.pose, 1.5, 5.0, self.zoom)
            draw_rectangle(img, self.previous_p, 1.5, 5.0, self.zoom)

        cv2.imshow('img', img)
        cv2.waitKey(10)


class CarmenEnv:
    def __init__(self, params):
        self.params = params
        carmen_path = os.environ['CARMEN_HOME']
        rddf_path = carmen_path + '/data/rndf/' + params['rddf']
        self.rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
        print('Connecting to carmen')
        carmen.init()

    def _read_state(self):
        carmen.handle_messages()
        laser = carmen.read_laser()

        state = {
            'pose': np.copy(carmen.read_truepos()),
            'laser': np.copy(laser).reshape(len(laser), 1),
        }

        return state

    def rear_laser_is_active(self):
        return carmen.config_rear_laser_is_active()

    def reset(self):
        carmen.publish_stop_command()

        max_pose_shift = 30
        min_pose_shift = 10

        init_pos_id = np.random.randint(max_pose_shift, len(self.rddf) - (max_pose_shift + 1))
        init_pos = self.rddf[init_pos_id]

        carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])

        if self.params['allow_negative_commands']: forw_or_back = np.random.randint(2) * 2 - 1
        else: forw_or_back = 1

        goal_id = init_pos_id + np.random.randint(min_pose_shift, max_pose_shift) * forw_or_back
        goal = self.rddf[goal_id]
        goal = goal[:4]
        self.goal = goal

        carmen.publish_goal_list([goal[0]], [goal[1]], [goal[2]], [goal[3]], [0.0], time.time())
        self.n_steps = 0

        return self._read_state(), goal

    def step(self, cmd):
        carmen.publish_goal_list([self.goal[0]], [self.goal[1]], [self.goal[2]], [self.goal[3]], [0.0], time.time())

        v = cmd[0] * 10.0
        phi = cmd[1] * np.deg2rad(28.)
        # v = 10.
        # phi = cmd[0] * np.deg2rad(28.0)

        carmen.publish_command([v] * 10, [phi] * 10, [0.1] * 10, True)

        state = self._read_state()

        achieved_goal = dist(state['pose'], self.goal) < self.params['goal_achievement_dist']
        vel_is_correct = np.abs(state['pose'][3] - self.goal[3]) < self.params['vel_achievement_dist']

        hit_obstacle = carmen.hit_obstacle()
        starved = self.n_steps >= self.params['n_steps_episode']
        success = achieved_goal  # and vel_is_correct

        done = success or hit_obstacle or starved
        info = {'success': success, 'hit_obstacle': hit_obstacle, 'starved': starved}

        self.n_steps += 1

        """
        if hit_obstacle:
            print('\n\n** HIT OBSTACLE\n\nLaser [size: ' + str(len(state['laser'])) + ']:\n')
            print(state['laser'])
            print('\n\n')
        """

        if done:
            carmen.publish_stop_command()

        return state, done, info

    def finalize(self):
        carmen.publish_stop_command()

    def view(self):
        pass



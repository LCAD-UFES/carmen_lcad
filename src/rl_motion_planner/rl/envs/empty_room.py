

import cv2
import numpy as np
from rl.envs import AbstractEnv
from rl.goal_samplers import PoseSampler
from rl.util import ackerman_motion_model, draw_rectangle, dist


class EmptyRoom(AbstractEnv):
    def __init__(self, params, room_size, viewer_size, pose2d_sampler, 
                 goal_achievement_dist, clip_pose):
        super().__init__(params, pose2d_sampler)
        self.room_size = room_size
        self.viewer_size = viewer_size  
        
        # add some extra space to prevent cutting the object in the corners 
        view_area_in_meters = (self.room_size + self.room_size * 0.1)
        self.pixels_by_meter = self.viewer_size / view_area_in_meters
        
        self.goal_achievement_dist = goal_achievement_dist
        self.clip_pose = clip_pose
        self.laser = np.zeros((1, 1))

    def _view(self):
        viewer = np.zeros((self.viewer_size, self.viewer_size, 3)) + 255

        # draw the reference system        
        p1 = (int(viewer.shape[1]/2), 0)
        p2 = (int(viewer.shape[1]/2), viewer.shape[0])
        cv2.line(viewer, p1, p2, (0, 255, 0))

        p1 = (0, int(viewer.shape[0]/2))
        p2 = (viewer.shape[1], int(viewer.shape[0]/2))
        cv2.line(viewer, p1, p2, (0, 255, 0))

        # draw the agent and the goal        
        self._object_drawer(viewer, self.goal, (255, 0, 0), self.pixels_by_meter)
        
        x = int(self.goal[0] * self.pixels_by_meter + viewer.shape[1] / 2)
        y = int(self.goal[1] * self.pixels_by_meter + viewer.shape[0] / 2)
        cv2.circle(viewer, (x, y), int(self.pixels_by_meter * self.goal_achievement_dist), (255, 0, 0), 1)

        self._object_drawer(viewer, self.pose, (0, 0, 255), self.pixels_by_meter)

        cv2.imshow("viewer", viewer)
        cv2.waitKey(1)

    def _reset_state(self):
        self.pose = np.zeros(5)
        self.state = {'pose': self.pose, 'laser': self.laser}
    
    def goal_reached(self, state, goal):
        return dist(state['pose'], goal) < self.goal_achievement_dist

    def _update_state(self, cmd):
        self.pose = self._motion_model(self.pose, cmd)
        
        # clip the position to prevent the agent from leaving 
        # the visible area.
        if self.clip_pose:
            s = self.room_size / 2
            self.pose[0] = np.clip(self.pose[0], -s, s)
            self.pose[1] = np.clip(self.pose[1], -s, s)

        self.state['pose'] = self.pose

    def _motion_model(self, pose, cmd):
        raise NotImplementedError()
    
    def _object_drawer(self, img, pose, color, pixels_by_meter):
        raise NotImplementedError()


class EmptyRoomSimple(EmptyRoom):
    def _motion_model(self, x, cmd):
        x[0] += cmd[0] * 0.5
        x[1] += cmd[1] * 0.5
        return x

    def _object_drawer(self, img, pose, color, pixels_by_meter):
        x = int(pose[0] * pixels_by_meter + img.shape[1] / 2)
        y = int(pose[1] * pixels_by_meter + img.shape[0] / 2)
        cv2.circle(img, (x, y), int(0.1 * pixels_by_meter), color, -1)

    def __init__(self, params):
        room_size = 10.0
        viewer_size = 300
        goal_achievement_dist = 0.5
        clip_pose = True
        pose2d_sampler = PoseSampler(2, room_size/2, room_size/2, 0., 0., 0., 0., distribution='uniform')
        self.env = super().__init__(params, room_size, viewer_size, pose2d_sampler, 
                                    goal_achievement_dist, clip_pose)


class EmptyRoomAckerman(EmptyRoom):
    def _motion_model(self, x, cmd):
            v = 0.5 * cmd[0]
            phi = np.deg2rad(28.) * cmd[1]
            return ackerman_motion_model(x, v, phi, dt=0.1, L=0.2625)
        
    def _object_drawer(self, img, pose, color, pixels_by_meter):
            draw_rectangle(img, pose, 0.15, 0.5, pixels_by_meter)

    def __init__(self, params):
        room_size = 20.0
        viewer_size = 500
        goal_achievement_dist = 0.5
        clip_pose = False
        pose2d_sampler = PoseSampler(2, room_size/2, room_size/2, 0., np.pi, np.pi, 0., distribution='uniform')
        self.env = super().__init__(params, room_size, viewer_size, pose2d_sampler, 
                                    goal_achievement_dist, clip_pose)





import os
import sys
import numpy as np
import time
import matplotlib.pyplot as plt
import carmen_comm.carmen_comm as carmen
from rl.util import Transform2d


def read_rddf(rddf):
    carmen_path = os.environ['CARMEN_HOME']
    # rddf_path = carmen_path + '/data/rndf/rddf-log_voltadaufes-20160513.txt'
    rddf_path = carmen_path + '/data/rndf/' + rddf
    rddf = [[float(field) for field in line.rstrip().rsplit(' ')] for line in open(rddf_path, 'r').readlines()]
    return rddf


def dist(pose, goal):
    return np.sqrt((pose[0] - goal[0]) ** 2 + (pose[1] - goal[1]) ** 2)


from gradmpp.scipy_planners import ScipyPolyPlanner
planner = ScipyPolyPlanner()


if True:
    goal = [0, 1, 0]
    goalv = 0.

    start = time.time()
    cmds, poses = planner.forward(goal, goalv)
    time_spent = time.time() - start

    cmds = np.reshape(cmds, [30, 2])
    poses = np.reshape(poses, [30, 3])

    d = np.sqrt((poses[-1, 0] - goal[0]) ** 2 + (poses[-1, 1] - goal[1]) ** 2)
    print('Time:', time_spent, 'Final Pose:', poses[-1], 'Dist to Goal:', d)

    plt.figure()
    plt.plot(poses[:, 0], poses[:, 1], '.-')
    plt.plot(goal[0], goal[1], 'ro-')

    plt.figure()
    plt.ylim(-10.5, 10.5)
    plt.plot(cmds[:, 0])

    plt.figure()
    plt.ylim(-28.5, 28.5)
    plt.plot(np.rad2deg(cmds[:, 1]))

    plt.show()

else:
    rddf = read_rddf('rddf-voltadaufes-20170809.txt')
    print("Carmen init")
    carmen.init()

    while True:
        init_pos_id = np.random.randint(len(rddf))
        init_pos = rddf[init_pos_id]

        print("Carmen reset pose")
        carmen.reset_initial_pose(init_pos[0], init_pos[1], init_pos[2])

        while True:
            carmen.handle_messages()
            pose_data = carmen.read_pose()
            goal_data = carmen.read_goal()

            goal_t = Transform2d(goal_data[0] - pose_data[0], goal_data[1] - pose_data[1], goal_data[2])
            pose_t = Transform2d(0., 0., pose_data[2])
            goal_t = pose_t.inverse().transform(goal_t)
            goal_t = [goal_t.x, goal_t.y, goal_t.th]

            print('Goal:', goal_t, 'GoalV:', goal_data[3])
            print("Planning...")
            cmds, poses = planner.forward(goal_t, goal_data[3])

            vs, ps, ts = [], [], []
            for v, phi in cmds:
                vs.append(float(v))
                ps.append(float(phi))
                ts.append(float(0.1))

            print('Publish', vs[0], ps[0], ts[0])
            carmen.publish_command(vs, ps, ts, True)

            if dist(goal_t, pose_data) < 0.5 and np.abs(goal_data[3] - pose_data[3]) < 0.5:
                print('success!')
                break
            elif carmen.hit_obstacle():
                print('fail!')
                break

        carmen.publish_stop_command()


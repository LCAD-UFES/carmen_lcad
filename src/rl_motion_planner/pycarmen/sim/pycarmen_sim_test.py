
import time
import numpy as np
from sim import CarmenSim  

sim = CarmenSim()

while True:
    sim.reset()

    while not sim.hit_obstacle():
        v = 10. * np.random.random()
        phi = np.deg2rad(28.) * (2. * np.random.random() - 1.)

        sim.step(v, phi, 0.1)
        
        pose = sim.pose()
        sim.draw_occupancy_map()
        sim.draw_pose(pose[0], pose[1], pose[2], 0, 0, 0)
        sim.draw_poses(sim.rddf_forward(), 0, 220, 220)
        sim.show(50)

        print()
        print("Pose:", sim.pose(), "time:", time.time())
        print("Laser:", sim.laser()[:20])
        print("Hit obstacle:", sim.hit_obstacle())
        print()


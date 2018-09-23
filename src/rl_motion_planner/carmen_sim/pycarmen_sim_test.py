
import time
import numpy as np
import pycarmen_sim as pycarmen

sim = pycarmen.CarmenSim()

while True:
    sim.reset()

    while not sim.hit_obstacle():
        v = 10. * np.random.random()
        phi = np.deg2rad(28.) * (2. * np.random.random() - 1.)

        sim.step(v, phi, 0.1)
        sim.view()

        print("Pose:", sim.pose(), "Goal:", sim.goal(), "time:", time.time())
        print("Laser:", sim.laser()[:20])
        print("Hit obstacle:", sim.hit_obstacle())


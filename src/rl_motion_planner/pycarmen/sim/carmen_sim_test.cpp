
#include "../sim/carmen_sim.h"

#include <cstdio>
#include <cstdlib>
#include <carmen/carmen.h>


int
main()
{
	CarmenSim sim;

	while (true)
	{
		sim.reset();

		while (!sim.hit_obstacle())
		{
			double v = 10. * 1; //((double) rand() / (double) RAND_MAX);
			double phi = carmen_degrees_to_radians(28.) * (2. * ((double) rand() / (double) RAND_MAX) - 1.);

			sim.step(v, phi, 0.1);

			printf("Pose: %.2lf %.2lf %.2lf %.2lf time: %lf\n",
					sim.pose()[0], sim.pose()[1], sim.pose()[2], sim.pose()[3],
					carmen_get_time());

			printf("Laser: ");

			for (int i = 0; i < 20; i++)
				printf("%.2lf ", sim.laser()[i]);

			printf("\n");
			printf("Hit obstacle: %d\n", sim.hit_obstacle());

			sim.draw_occupancy_map();
			sim.draw_pose(sim.pose()[0], sim.pose()[1], sim.pose()[2], 0, 0, 0);
			sim.draw_poses(sim.rddf_forward(), 0, 200, 200);
			sim.show(50);
		}
	}

	return 0;
}

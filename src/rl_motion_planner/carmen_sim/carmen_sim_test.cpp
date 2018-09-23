
#include <cstdio>
#include <cstdlib>
#include <carmen/carmen.h>
#include <carmen/carmen_sim.h>


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

			printf("Pose: %.2lf %.2lf %.2lf %.2lf Goal: %.2lf %.2lf %.2lf %.2lf time: %lf\n",
					sim.pose()[0], sim.pose()[1], sim.pose()[2], sim.pose()[3],
					sim.goal()[0], sim.goal()[1], sim.goal()[2], sim.goal()[3],
					carmen_get_time());

			printf("Laser: ");

			for (int i = 0; i < 20; i++)
				printf("%.2lf ", sim.laser()[i]);

			printf("\n");
			printf("Hit obstacle: %d\n", sim.hit_obstacle());

			sim.view();
		}
	}

	return 0;
}

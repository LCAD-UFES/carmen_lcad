
#include <cstdio>
#include <vector>
#include <unistd.h>
#include "carmen_comm.h"
using namespace std;


int
main()
{
	init();

	while (1)
	{
		while (not hit_obstacle())
		{
			printf("reseting pose\n");
			reset_initial_pose(7757671.481041, -363606.265849, 0.682322);

			sleep(1);
			// handle_messages(0.1);

			vector<double> vs, phis, ts;

			for (int i = 0; i < 30; i++)
			{
				vs.push_back(10.0);
				phis.push_back(0.);
				ts.push_back(0.1);
			}

			printf("Publish command\n");
			publish_command(vs, phis, ts, true, 7757671.481041, -363606.265849, 0.682322);
		}

		publish_stop_command();
	}

	return 0;
}

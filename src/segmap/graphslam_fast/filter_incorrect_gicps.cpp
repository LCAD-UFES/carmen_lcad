
#include <carmen/segmap_dataset.h>
#include <carmen/segmap_util.h>
#include "g2o/types/slam2d/se2.h"

using namespace g2o;


class LoopRestriction
{
	public:
		SE2 transform;
		int converged;
		int from;
		int to;
};


void
read_loop_restrictions(char *filename, vector<LoopRestriction> &loop_data)
{
	int n;
	FILE *f;
	double x, y, theta;

	if ((f = fopen(filename, "r")) == NULL)
	{
		printf("Warning: Unable to open file '%s'! Ignoring loop closures.\n", filename);
		return;
	}

	while(!feof(f))
	{
		LoopRestriction l;

		n = fscanf(f, "%d %d %d %lf %lf %lf\n",
			&l.from, &l.to, &l.converged, &x, &y, &theta
		);

		if (n == 6)
		{
			l.transform = SE2(x, y, theta);
			loop_data.push_back(l);
		}
	}

	fclose(f);
}


int 
main(int argc, char **argv)
{
    if (argc < 3)
        exit(printf("Error: Use %s <dataset_dir> <output_name>\n", argv[0]));

    char path[256];
    DatasetCarmen dataset(argv[1], 0);
    vector<LoopRestriction> gicp_gps;
    vector<LoopRestriction> gicp_gps_filtered;

    sprintf(path, "%s/gicp_to_map_graphslam.txt", argv[1]);
    read_loop_restrictions(path, gicp_gps);
	
    for (int i = 0; i < gicp_gps.size(); i++)
    {
        int id = gicp_gps[i].from;
        
        // angle significantly different from the one from graphslam
        if (fabs(g2o::normalize_theta(gicp_gps[i].transform[2] - dataset.data[id].pose.th)) > degrees_to_radians(10))
            continue;
        
        // distance significantly different from graphslam 
        if (dist2d(gicp_gps[i].transform[0], gicp_gps[i].transform[1], dataset.data[id].pose.x, dataset.data[id].pose.y) > 3.0)
            continue;

        if (i > 0)
        {
            if (gicp_gps[i-1].from == id - 1)
            {
                // big shift in y direction (graphslam)
                SE2 p(dataset.data[id].pose.x, dataset.data[id].pose.y, dataset.data[id].pose.th);
                if (fabs((p.inverse() * gicp_gps[i].transform)[1]) > 1.0)
                    continue;

                // big shift in y direction (previous pose)
                if (fabs((gicp_gps[i-1].transform.inverse() * gicp_gps[i].transform)[1]) > 1.0)
                    continue;

                // velocity inconsistent with odometry
                if ((gicp_gps[i-1].transform.inverse() * gicp_gps[i].transform)[0] * dataset.data[id].v < 0.)
                    continue;
            }
        }

        gicp_gps_filtered.push_back(gicp_gps[i]);
    }

    printf("N discarded gicps: %d N remaining: %d\n", 
        gicp_gps.size() - gicp_gps_filtered.size(), gicp_gps_filtered.size()
    );

    FILE *f = fopen(argv[2], "w");

    for (int i = 0; i < gicp_gps_filtered.size(); i++)
    {
        fprintf(f, "%d %d %d %lf %lf %lf\n", 
            gicp_gps_filtered[i].from, gicp_gps_filtered[i].to, gicp_gps_filtered[i].converged,
            gicp_gps_filtered[i].transform[0], gicp_gps_filtered[i].transform[1], gicp_gps_filtered[i].transform[2]
        );
    }

    fclose(f);
    return 0;
}


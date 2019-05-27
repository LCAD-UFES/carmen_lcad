#include <carmen/carmen.h>
#include <tf.h>
#include <Eigen/Core>
#include "g2o/types/slam2d/se2.h"
#include <vector>

using namespace std;

typedef struct
{
	carmen_pose_3D_t odometry_pose;
	carmen_pose_3D_t gps_pose;
	carmen_pose_3D_t icp_pose;
	double gps_std;
	double gps_yaw;
	double gps_orientation_valid;
	double timestamp;
} Line;

typedef struct
{
	uint id;
	carmen_pose_3D_t globalpos;
	double v;
	double phi;
	double timestamp;
	double delta_x;
	double delta_y;
} GlobalPos;

typedef struct
{
	uint i;
	uint loop_id;

	uint from_idx;
	uint to_idx;
} LoopPair;

FILE *output_file;


Line
read_sync_line(FILE *f)
{
	Line l;

	memset(&l.odometry_pose, 0, sizeof(l.odometry_pose));
	memset(&l.gps_pose, 0, sizeof(l.gps_pose));
	memset(&l.icp_pose, 0, sizeof(l.icp_pose));

	fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
		&(l.odometry_pose.position.x), &(l.odometry_pose.position.y), &(l.odometry_pose.orientation.yaw),
		&(l.gps_pose.position.x), &(l.gps_pose.position.y), &(l.gps_pose.orientation.yaw),
		&(l.icp_pose.position.x), &(l.icp_pose.position.y), &(l.icp_pose.orientation.yaw),
		&l.timestamp, &l.gps_std, &l.gps_yaw, &l.gps_orientation_valid
	);

//	printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
//		l.odometry_pose.position.x, l.odometry_pose.position.y, l.odometry_pose.orientation.yaw,
//		l.gps_pose.position.x, l.gps_pose.position.y, l.gps_pose.orientation.yaw,
//		l.icp_pose.position.x, l.icp_pose.position.y, l.icp_pose.orientation.yaw,
//		l.timestamp
//	);

	return (l);
}


GlobalPos
read_globalpos_line(FILE *f)
{
	GlobalPos l;

	memset(&l.globalpos, 0, sizeof(l.globalpos));

	fscanf(f, "%lf %lf %lf %lf %lf %lf %lf\n",
		&(l.globalpos.position.x), &(l.globalpos.position.y), &(l.globalpos.position.z),
		&(l.globalpos.orientation.yaw),
		&(l.v), &(l.phi), &l.timestamp
	);

//	printf("%lf %lf %lf %lf %lf %lf %lf\n",
//		l.globalpos.position.x, l.globalpos.position.y, l.globalpos.position.z,
//		l.globalpos.orientation.yaw,
//		l.v, l.phi, l.timestamp
//	);

	return (l);
}


vector<Line>
read_sync_file(char *input_file)
{
	FILE *f = fopen(input_file, "r");

	if (f == NULL)
		exit(printf("File '%s' could not be open\n", input_file));

	vector<Line> input_data;
	while(!feof(f))
	{
		Line l = read_sync_line(f);
		input_data.push_back(l);
	}

	fclose(f);

	return (input_data);
}


vector<GlobalPos>
read_globalpos_file(char *input_file)
{
	FILE *f = fopen(input_file, "r");

	if (f == NULL)
		exit(printf("File '%s' could not be open\n", input_file));

	vector<GlobalPos> input_data;
	while(!feof(f))
	{
		GlobalPos l = read_globalpos_line(f);
		input_data.push_back(l);
	}

	fclose(f);

	return (input_data);
}


void
get_ids(vector<GlobalPos> &globapos, vector<Line> sync_data)
{
	uint id = 0;
	for (uint i = 0; i < globapos.size(); i++)
	{
		for ( ; id < sync_data.size(); id++)
		{
			if (globapos[i].timestamp == sync_data[id].timestamp)
			{
				globapos[i].id = id;
				break;
			}
		}

		if (id == sync_data.size())
		{
			printf("globalpos %d was not found in sync file\n", i);
			id = 0;
		}
//		else
//		{
//			GlobalPos l = globapos[i];
//			printf("%lf %lf %lf %lf %lf %lf %lf %d\n",
//				l.globalpos.position.x, l.globalpos.position.y, l.globalpos.position.z,
//				l.globalpos.orientation.yaw,
//				l.v, l.phi, l.timestamp, l.id
//			);
//		}
	}
}


vector<LoopPair>
find_loop_pairs(vector<GlobalPos> &globapos_from, vector<GlobalPos> globapos_to)
{
	uint globapos_to_loop_id, to_idx;
	vector<LoopPair> loop_pair_v;

	globapos_to_loop_id = to_idx = 0;
	for (uint i = 0; i < globapos_from.size(); i++)
	{
		double min_dist = DBL_MAX;

		for (uint j = 0; j < globapos_to.size(); j++)
		{
			double delta_x = globapos_to[j].globalpos.position.x - globapos_from[i].globalpos.position.x;
			double delta_y = globapos_to[j].globalpos.position.y - globapos_from[i].globalpos.position.y;

			double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

			if (dist < min_dist)
			{
				min_dist = dist;
				globapos_to_loop_id = globapos_to[j].id;
				to_idx = j;

				globapos_from[i].delta_x = delta_x;
				globapos_from[i].delta_y = delta_y;
			}
		}

		LoopPair loop_pair = {globapos_from[i].id, globapos_to_loop_id, i, to_idx};
		loop_pair_v.push_back(loop_pair);
	}
	return (loop_pair_v);
}


void
print_loop_pairs(vector<GlobalPos> globapos_from, vector<GlobalPos> globapos_to)
{
	FILE *from = fopen("from.txt", "w");
	FILE *to = fopen("to.txt", "w");

	for (uint i = 0; i < globapos_from.size(); i++)
		fprintf(from, "%lf %lf %lf %lf\n", globapos_from[i].globalpos.position.x, globapos_from[i].globalpos.position.y,
				globapos_from[i].delta_x, globapos_from[i].delta_y);

	for (uint i = 0; i < globapos_to.size(); i++)
		fprintf(to, "%lf %lf\n", globapos_to[i].globalpos.position.x, globapos_to[i].globalpos.position.y);

	fclose(from);
	fclose(to);
}


Eigen::Matrix<double, 4, 4>
transform_carmen_pose_to_pcl_pose(carmen_pose_3D_t *carmen_pose)
{
	Eigen::Matrix<double, 4, 4> pcl_pose;

	tf::Quaternion quat(carmen_pose->orientation.yaw,
					carmen_pose->orientation.pitch,
					carmen_pose->orientation.roll);

	tf::Matrix3x3 rotation(quat);

	pcl_pose(0, 0) = rotation[0][0];
	pcl_pose(0, 1) = rotation[0][1];
	pcl_pose(0, 2) = rotation[0][2];
	pcl_pose(1, 0) = rotation[1][0];
	pcl_pose(1, 1) = rotation[1][1];
	pcl_pose(1, 2) = rotation[1][2];
	pcl_pose(2, 0) = rotation[2][0];
	pcl_pose(2, 1) = rotation[2][1];
	pcl_pose(2, 2) = rotation[2][2];

	pcl_pose(0, 3) = carmen_pose->position.x;
	pcl_pose(1, 3) = carmen_pose->position.y;
	pcl_pose(2, 3) = carmen_pose->position.z;

	pcl_pose(3, 0) = 0;
	pcl_pose(3, 1) = 0;
	pcl_pose(3, 2) = 0;
	pcl_pose(3, 3) = 1;

	return (pcl_pose);
}


void
transform_pcl_pose_to_carmen_pose(Eigen::Matrix<double, 4, 4> pcl_pose, carmen_pose_3D_t *carmen_pose)
{
	tf::Matrix3x3 rotation;
	double roll, pitch, yaw;

	rotation[0][0] = pcl_pose(0, 0);
	rotation[0][1] = pcl_pose(0, 1);
	rotation[0][2] = pcl_pose(0, 2);
	rotation[1][0] = pcl_pose(1, 0);
	rotation[1][1] = pcl_pose(1, 1);
	rotation[1][2] = pcl_pose(1, 2);
	rotation[2][0] = pcl_pose(2, 0);
	rotation[2][1] = pcl_pose(2, 1);
	rotation[2][2] = pcl_pose(2, 2);

	rotation.getRPY(roll, pitch, yaw);

	carmen_pose->orientation.roll = roll;
	carmen_pose->orientation.pitch = pitch;
	carmen_pose->orientation.yaw = yaw;
	carmen_pose->position.x = pcl_pose(0, 3);
	carmen_pose->position.y = pcl_pose(1, 3);
	carmen_pose->position.z = pcl_pose(2, 3);
}


void
add_graphslam_restriction_to_output_file(int from, int to, int sync_file_line_from, int sync_file_line_to,
		vector<GlobalPos> globapos_from, vector<GlobalPos> globapos_to)
{
	carmen_pose_3D_t pose_i = globapos_from[from].globalpos;
	pose_i.position.x -= globapos_from[0].globalpos.position.x;	// primeira globapos_from é a origem (para reduzir problemas numéricos)
	pose_i.position.y -= globapos_from[0].globalpos.position.y;
	Eigen::Matrix<double, 4, 4> pose_i_eigen = transform_carmen_pose_to_pcl_pose(&pose_i);

	carmen_pose_3D_t pose_j = globapos_to[to].globalpos;
	pose_j.position.x -= globapos_from[0].globalpos.position.x;	// primeira globapos_from é a origem (para reduzir problemas numéricos)
	pose_j.position.y -= globapos_from[0].globalpos.position.y;
	Eigen::Matrix<double, 4, 4> pose_j_eigen = transform_carmen_pose_to_pcl_pose(&pose_j);

	Eigen::Matrix<double, 4, 4> i_to_j_tranform = (pose_i_eigen.inverse() * pose_j_eigen).cast<double>();

	carmen_pose_3D_t carmen_transform;
	transform_pcl_pose_to_carmen_pose(i_to_j_tranform, &carmen_transform);

	fprintf(output_file, "%d %d %lf %lf %lf\n", sync_file_line_from, sync_file_line_to, carmen_transform.position.x, carmen_transform.position.y, carmen_transform.orientation.yaw);
	fflush(output_file);
}


void
process_data(vector<Line> sync_data, vector<GlobalPos> globapos_from, vector<GlobalPos> globapos_to)
{
	printf("Processing globalpos_from file.\n");
	get_ids(globapos_from, sync_data);

	printf("Processing globalpos_to file.\n");
	get_ids(globapos_to, sync_data);

	vector<LoopPair> loop_pairs = find_loop_pairs(globapos_from, globapos_to);


	uint loop_id;
	int pairs_added = 0;
	for (uint k = 0; k < loop_pairs.size(); k++)
	{
		uint i = loop_pairs[k].i;
		loop_id = loop_pairs[k].loop_id;

		if (((k + 1) < (loop_pairs.size() - 1)) && (loop_id == loop_pairs[k + 1].loop_id))
		{
			globapos_from[loop_pairs[k].from_idx].delta_x = globapos_from[loop_pairs[k].from_idx].delta_y = 0.0; // usados soh para visualizacao em print_loop_pairs()
			continue; // remove duplicated pairs
		}

		add_graphslam_restriction_to_output_file(loop_pairs[k].from_idx, loop_pairs[k].to_idx, i, loop_id, globapos_from, globapos_to);

		pairs_added++;
	}

	printf("num loops found: %ld; num loops used: %d\n", loop_pairs.size(), pairs_added);
	print_loop_pairs(globapos_from, globapos_to);
}


int
main(int argc, char **argv)
{
	if (argc < 4)
		exit(printf("Use %s <sync-file> <globalpos-from-file> <globalpos-to-file> <output-file>\n", argv[0]));

	vector<Line> sync_data = read_sync_file(argv[1]);
	vector<GlobalPos> globapos_from = read_globalpos_file(argv[2]);
	vector<GlobalPos> globapos_to = read_globalpos_file(argv[3]);

	output_file = fopen(argv[4], "w");
	if (output_file == NULL)
		exit(printf("Unable to open file '%s'", argv[4]));

	process_data(sync_data, globapos_from, globapos_to);

	fclose(output_file);

	printf("Pressione crtl+c para terminar.\n");
	fflush(stdout);
	getchar();

	return (0);
}

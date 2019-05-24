#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include "g2o/types/slam2d/se2.h"

#include <tf.h>
#include <vector>
#include <carmen/carmen.h>

using namespace std;
using namespace g2o;

typedef struct
{
	carmen_pose_3D_t odometry_pose;
	carmen_pose_3D_t gps_pose;
	carmen_pose_3D_t icp_pose;
	double gps_std;
	double gps_yaw;
	double gps_orientation_valid;
	double timestamp;
}Line;

typedef struct
{
	uint i;
	int loop_id;
} LoopPair;

char *velodyne_storage_dir;
pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
vector<Line> input_data;


char *output_filename;
FILE *output_file;


Line
read_line(FILE *f)
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

	return l;
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

	return pcl_pose;
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
save_clouds_for_debug(pcl::PointCloud<pcl::PointXYZRGB> source_pointcloud, pcl::PointCloud<pcl::PointXYZRGB> target_pointcloud,
		pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud_transformed, char forward_backward)
{
	static int id_icp_call = 1;
	char name[1024];

	sprintf(name, "%s/%05d_source_%c.pcd", velodyne_storage_dir, id_icp_call, forward_backward);
	for (uint i = 0; i < (source_pointcloud.width * source_pointcloud.height); i++)
		source_pointcloud.points[i].r = 255;
	pcl::io::savePCDFileASCII(name, source_pointcloud);

	sprintf(name, "%s/%05d_target_%c.pcd", velodyne_storage_dir, id_icp_call, forward_backward);
	for (uint i = 0; i < (target_pointcloud.width * target_pointcloud.height); i++)
		target_pointcloud.points[i].g = 255;
	pcl::io::savePCDFileASCII(name, target_pointcloud);

	sprintf(name, "%s/%05d_result_%c.pcd", velodyne_storage_dir, id_icp_call, forward_backward);
	for (uint i = 0; i < (out_pcl_pointcloud_transformed.width * out_pcl_pointcloud_transformed.height); i++)
		out_pcl_pointcloud_transformed.points[i].b = 255;
	pcl::io::savePCDFileASCII(name, out_pcl_pointcloud_transformed);

	printf("Saved: %s\n", name);

	id_icp_call++;
}


double
average_distance_between_point_clouds(pcl::PointCloud<pcl::PointXYZRGB> pc_a, pcl::PointCloud<pcl::PointXYZRGB> pc_b)
{
	double sum = 0.0;

	for (uint i = 0; i < pc_a.size(); i++)
		sum += sqrt(pow(pc_a.points[i].x - pc_b.points[i].x, 2.0) +
				pow(pc_a.points[i].y - pc_b.points[i].y, 2.0) +
				pow(pc_a.points[i].z - pc_b.points[i].z, 2.0));

	return (sum / (double) pc_a.size());
}


int
perform_icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr j_pointcloud, Eigen::Matrix<double, 4, 4> *icp_i_to_j_transform)
{
	pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud_transformed_forward;
	pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud_transformed_backward;

	if ((i_pointcloud->size() >= 20) && (j_pointcloud->size() >= 20))
	{
		Eigen::Matrix<double, 4, 4> forward_transform;
		Eigen::Matrix<double, 4, 4> backward_transform;

		gicp.setInputCloud(i_pointcloud);
		gicp.setInputTarget(j_pointcloud);
		gicp.align(out_pcl_pointcloud);
		if (gicp.hasConverged())
		{
			forward_transform = gicp.getFinalTransformation().cast<double>();
			pcl::transformPointCloud(*i_pointcloud, out_pcl_pointcloud_transformed_forward, forward_transform);
			save_clouds_for_debug(*i_pointcloud, *j_pointcloud, out_pcl_pointcloud_transformed_forward, 'f');
		}
		else
			return (0);
		gicp.setInputCloud(j_pointcloud);
		gicp.setInputTarget(i_pointcloud);
		gicp.align(out_pcl_pointcloud);
		if (gicp.hasConverged())
		{
			backward_transform = gicp.getFinalTransformation().inverse().cast<double>();
			pcl::transformPointCloud(*i_pointcloud, out_pcl_pointcloud_transformed_backward, backward_transform);
			save_clouds_for_debug(*i_pointcloud, *j_pointcloud, out_pcl_pointcloud_transformed_backward, 'b');
		}
		else
			return (0);

		double average_distance = average_distance_between_point_clouds(out_pcl_pointcloud_transformed_forward, out_pcl_pointcloud_transformed_backward);
		printf("average_distance: %lf\n", average_distance);

		if (average_distance < 1.0)
		{
			(*icp_i_to_j_transform) = backward_transform; // a backaward é melhor no aeroporto gpx

			return (1);
		}
	}

	return (0);
}


void
load_pointcloud_from_file(char *filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
	int r, g, b;
	long int num_points;
	pcl::PointXYZRGB p3D;

	pointcloud->clear();

	FILE *f = fopen(filename, "r");

	if (f == NULL)
	{
		carmen_warn("Error: The pointcloud '%s' not exists!\n", filename);
		return;
	}

	fscanf(f, "%ld\n", &num_points);

	for (long i = 0; i < num_points; i++)
	{
		fscanf(f, "%f %f %f %d %d %d\n",
			&p3D.x, &p3D.y, &p3D.z,
			&r, &g, &b
		);

		p3D.r = (unsigned char) r;
		p3D.g = (unsigned char) g;
		p3D.b = (unsigned char) b;

		if ((p3D.z > 0.0) && (sqrt(p3D.x * p3D.x + p3D.y * p3D.y) < 30.0))
			pointcloud->push_back(p3D);
	}

	fclose(f);
}


void
initialize_icp()
{
	gicp.setMaximumIterations(2000);
	//gicp.setMaximumIterations(500);
	gicp.setTransformationEpsilon(1e-3);
	//gicp.setRotationEpsilon(1e-3);
	gicp.setMaxCorrespondenceDistance(15.0);
	//gicp.setMaxCorrespondenceDistance(10.0);
//	gicp.setRANSACOutlierRejectionThreshold(1.0);
}


void
read_data(char *input_file)
{
	FILE *f = fopen(input_file, "r");

	if (f == NULL)
		exit(printf("File '%s' could not be open\n", input_file));

	while(!feof(f))
	{
		Line l = read_line(f);
		input_data.push_back(l);
	}

	fclose(f);
}


void
add_gps_restriction(int i, int j)
{
	SE2 gps_from(input_data[i].gps_pose.position.x, input_data[i].gps_pose.position.y, input_data[i].gps_pose.orientation.yaw);
	SE2 gps_to(input_data[j].gps_pose.position.x, input_data[j].gps_pose.position.y, input_data[j].gps_pose.orientation.yaw);
	SE2 transf = gps_from.inverse() * gps_to;

	printf("%d %d %lf %lf %lf\n", i, j, transf[0], transf[1], transf[2]);
}


void
project_to_world(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_world, int i)
{
	carmen_pose_3D_t pose_translated = input_data[i].gps_pose;

	pose_translated.position.x -= input_data[0].gps_pose.position.x;
	pose_translated.position.y -= input_data[0].gps_pose.position.y;

	Eigen::Matrix<double, 4, 4> transform = transform_carmen_pose_to_pcl_pose(&pose_translated);
	pcl::transformPointCloud(*pointcloud, *pointcloud_world, transform);
}


static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
LeafSize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud, double size)
{
       pcl::VoxelGrid<pcl::PointXYZRGB> grid;
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointCloud  (new pcl::PointCloud<pcl::PointXYZRGB>);
       grid.setLeafSize(size, size, size);
       grid.setInputCloud(inputPointCloud);
       grid.filter(*outputPointCloud);

       return (outputPointCloud);
}


bool
add_icp_restriction(int i, int j)
{
	char filename_t0[1024];
	char filename_t1[1024];

	static int is_first = 1;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pointcloud;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr j_pointcloud;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pointcloud_in_j_coordinate_system;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr i_pointcloud_leafed, j_pointcloud_leafed;

	if (is_first)
	{
		i_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		j_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		i_pointcloud_in_j_coordinate_system = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);

		is_first = 0;
	}

	sprintf(filename_t0, "%s/%lf.ply", velodyne_storage_dir, input_data[i].timestamp);
	sprintf(filename_t1, "%s/%lf.ply", velodyne_storage_dir, input_data[j].timestamp);

	load_pointcloud_from_file(filename_t0, i_pointcloud);
	load_pointcloud_from_file(filename_t1, j_pointcloud);

	i_pointcloud_leafed = LeafSize(i_pointcloud, 0.2);
	j_pointcloud_leafed = LeafSize(j_pointcloud, 0.2);
	printf("size_in %ld, size_out %ld\n", i_pointcloud->size(), i_pointcloud_leafed->size());

	if (i_pointcloud->size() <= 0)
		exit(printf("Error: SourceCloud empty: %s\n", filename_t0));

	if (j_pointcloud->size() <= 0)
		exit(printf("Error: TargetCloud empty: %s\n", filename_t1));

//	project_to_world(i_pointcloud_leafed, i_pointcloud_in_j_coordinate_system, i);
	//project_to_world(j_pointcloud_leafed, j_pointcloud_world, j);

	carmen_pose_3D_t pose_i = input_data[i].gps_pose;
	pose_i.position.x -= input_data[0].gps_pose.position.x;	// primeira pose do gps é a origem (para reduzir problemas numéricos)
	pose_i.position.y -= input_data[0].gps_pose.position.y;
	Eigen::Matrix<double, 4, 4> pose_i_eigen = transform_carmen_pose_to_pcl_pose(&pose_i);

	carmen_pose_3D_t pose_j = input_data[j].gps_pose;
	pose_j.position.x -= input_data[0].gps_pose.position.x;	// primeira pose do gps é a origem (para reduzir problemas numéricos)
	pose_j.position.y -= input_data[0].gps_pose.position.y;
	Eigen::Matrix<double, 4, 4> pose_j_eigen = transform_carmen_pose_to_pcl_pose(&pose_j);

	Eigen::Matrix<double, 4, 4> i_to_j_tranform = (pose_j_eigen.inverse() * pose_i_eigen).cast<double>();
	// Desloca os pontos da point cloud i da diferença entre as poses de i e j (assumindo que j é origem da deferença) 
	pcl::transformPointCloud(*i_pointcloud_leafed, *i_pointcloud_in_j_coordinate_system, i_to_j_tranform);

	bool pair_added = false;
	Eigen::Matrix<double, 4, 4> icp_i_to_j_transform;
	if (perform_icp(i_pointcloud_in_j_coordinate_system, j_pointcloud_leafed, &icp_i_to_j_transform))
	{
		Eigen::Matrix<double, 4, 4> eigen_tranform = (icp_i_to_j_transform * pose_j_eigen.inverse() * pose_i_eigen).inverse();
		carmen_pose_3D_t carmen_transform;
		transform_pcl_pose_to_carmen_pose(eigen_tranform, &carmen_transform);

		fprintf(output_file, "%d %d %lf %lf %lf\n", i, j, carmen_transform.position.x, carmen_transform.position.y, carmen_transform.orientation.yaw);
		fflush(output_file);

		pair_added = true;
	}
	else
		fprintf(stderr, "Not converged!\n");

	i_pointcloud->clear();
	j_pointcloud->clear();

	i_pointcloud_in_j_coordinate_system->clear();

	i_pointcloud_leafed->clear();
	j_pointcloud_leafed->clear();

	return (pair_added);
}


void
process_data(double dist_for_detecting_loop_closure, double time_difference_for_detecting_loop_closure)
{
	uint i, j;
	int loop_id;
	vector<LoopPair> loop_pair_v;

	for (i = 0; i < input_data.size(); i++)
	{
		loop_id = -1;
		double min_dist = DBL_MAX;

		for (j = (i + 1); j < input_data.size(); j++)
		{
			double delta_x = input_data[i].gps_pose.position.x - input_data[j].gps_pose.position.x;
			double delta_y = input_data[i].gps_pose.position.y - input_data[j].gps_pose.position.y;
			double delta_t = input_data[i].timestamp - input_data[j].timestamp;

			double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

			if ((dist < min_dist) && (fabs(delta_t) > time_difference_for_detecting_loop_closure)) // Tempo e distancia para detectar fechamento de loop. Alberto: TODO - passar o tempo como parametro.
			{
				min_dist = dist;
				loop_id = j;
			}
		}

		if (min_dist < dist_for_detecting_loop_closure)
		{
			LoopPair loop_pair = {i, loop_id};
			loop_pair_v.push_back(loop_pair);
		}
	}

	int pairs_added = 0;
	for (uint k = 0; k < loop_pair_v.size(); k++)
	{
		i = loop_pair_v[k].i;
		loop_id = loop_pair_v[k].loop_id;
		fprintf(stderr, "Loop pair (%d -> %d); %d de %ld (%.1lf%%)\n",
				i, loop_id, i, loop_pair_v.size(), 100.0 * (double) i / (double) loop_pair_v.size());

		if (((k + 1) < (loop_pair_v.size() - 1)) && (loop_id == loop_pair_v[k + 1].loop_id))
			continue; // remove duplicated pairs

		if (add_icp_restriction(i, loop_id))
			pairs_added++;
	}

	printf("num loops found: %ld; num loops used: %d\n", loop_pair_v.size(), pairs_added);
}


void
clean_data()
{
	fclose(output_file);
}


int
main(int argc, char **argv)
{
	if (argc < 4)
	{
		printf("Use %s <input-file> <velodyne-dir> <output_file> <dist_for_detecting_loop_closure (meters)> <time_difference_for_detecting_loop_closure (seconds)>\n", argv[0]);
		exit(1);
	}

	double dist_for_detecting_loop_closure = 4.0;
	double time_difference_for_detecting_loop_closure = 120.0;

	char *input_file = argv[1];

	velodyne_storage_dir = argv[2];

	output_file = fopen(argv[3], "w");
	if (output_file == NULL)
		exit(printf("Unable to open file '%s'", output_filename));

	if (argc >= 5)
		dist_for_detecting_loop_closure = atof(argv[4]);

	if (argc >= 6)
		time_difference_for_detecting_loop_closure = atof(argv[5]);

	initialize_icp();
	read_data(input_file);
	process_data(dist_for_detecting_loop_closure, time_difference_for_detecting_loop_closure);

	fclose(output_file);

	printf("Pressione crtl+c para terminar.\n");
	fflush(stdout);
	getchar();

	return 0;
}

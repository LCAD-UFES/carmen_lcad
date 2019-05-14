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

char *velodyne_storage_dir;
// "/media/OS/Users/Filipe/Desktop/log_voltadaufes-20140320-3.txt/";
// char *velodyne_storage_dir = "/media/OS/Users/Filipe/Desktop/ramdisk";
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
save_clouds_for_debug(pcl::PointCloud<pcl::PointXYZRGB> source_pointcloud, pcl::PointCloud<pcl::PointXYZRGB> target_pointcloud, pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud_transformed)
{
	static int id_icp_call = 1;
	char name[1024];

	sprintf(name, "%s/%05d_source.ply", velodyne_storage_dir, id_icp_call);
	pcl::io::savePLYFile(name, source_pointcloud);
	sprintf(name, "%s/%05d_target.ply", velodyne_storage_dir, id_icp_call);
	pcl::io::savePLYFile(name, target_pointcloud);
	sprintf(name, "%s/%05d_result.ply", velodyne_storage_dir, id_icp_call);
	pcl::io::savePLYFile(name, out_pcl_pointcloud_transformed);
	printf("Saved: %s\n", name);

	id_icp_call++;
}


int
perform_icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pointcloud, Eigen::Matrix<double, 4, 4> *measured_pose_out)
{
	pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud_transformed;

	gicp.setInputCloud(source_pointcloud);
	gicp.setInputTarget(target_pointcloud);
	gicp.align(out_pcl_pointcloud);
	(*measured_pose_out) = gicp.getFinalTransformation().cast<double>();

	pcl::transformPointCloud(out_pcl_pointcloud, out_pcl_pointcloud_transformed, (*measured_pose_out));
	// save_clouds_for_debug(*source_pointcloud, *target_pointcloud, out_pcl_pointcloud_transformed);

	if(gicp.hasConverged())
	{
		return 1;
	}

	return 0;
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


carmen_pose_3D_t
get_loop_closure_transform(int i, int j, Eigen::Matrix<double, 4, 4> correction)
{
	carmen_pose_3D_t posei_translated = input_data[i].gps_pose;
	carmen_pose_3D_t posej_translated = input_data[j].gps_pose;
	carmen_pose_3D_t transform;

	posei_translated.position.x -= input_data[0].gps_pose.position.x;
	posei_translated.position.y -= input_data[0].gps_pose.position.y;

	posej_translated.position.x -= input_data[0].gps_pose.position.x;
	posej_translated.position.y -= input_data[0].gps_pose.position.y;

	Eigen::Matrix<double, 4, 4> T;
	Eigen::Matrix<double, 4, 4> source = transform_carmen_pose_to_pcl_pose(&posei_translated);
	Eigen::Matrix<double, 4, 4> target = transform_carmen_pose_to_pcl_pose(&posej_translated);

	T = (source.inverse() * correction.inverse() * target);
	transform_pcl_pose_to_carmen_pose(T, &transform);

	return transform;
}


static pcl::PointCloud<pcl::PointXYZRGB>::Ptr
LeafSize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud, double size)
{
       pcl::VoxelGrid<pcl::PointXYZRGB> grid;
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointCloud  (new pcl::PointCloud<pcl::PointXYZRGB>);
       grid.setLeafSize(size, size, size);
       grid.setInputCloud(inputPointCloud);
       grid.filter(*outputPointCloud);
       return outputPointCloud;
}


void
add_icp_restriction(int i, int j)
{
	char filename_t0[1024];
	char filename_t1[1024];
	Eigen::Matrix<double, 4, 4> measured_pose_out;
	Eigen::Matrix<double, 4, 4> transform;

	static int is_first = 1;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pointcloud;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud_world;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pointcloud_world , source_pointcloud_leafed, target_pointcloud_leafed;

	if (is_first)
	{
		source_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		target_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		source_pointcloud_world = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		target_pointcloud_world = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);

		is_first = 0;
	}

	sprintf(filename_t0, "%s/%lf.ply", velodyne_storage_dir, input_data[i].timestamp);
	sprintf(filename_t1, "%s/%lf.ply", velodyne_storage_dir, input_data[j].timestamp);

	load_pointcloud_from_file(filename_t0, source_pointcloud);
	load_pointcloud_from_file(filename_t1, target_pointcloud);

	source_pointcloud_leafed = LeafSize(source_pointcloud, 0.2);
	target_pointcloud_leafed = LeafSize(target_pointcloud, 0.2);

	if (source_pointcloud->size() <= 0)
		exit(printf("Error: SourceCloud empty: %s\n", filename_t0));

	if (target_pointcloud->size() <= 0)
		exit(printf("Error: TargetCloud empty: %s\n", filename_t1));

//	project_to_world(source_pointcloud_leafed, source_pointcloud_world, i);
	//project_to_world(target_pointcloud_leafed, target_pointcloud_world, j);

	carmen_pose_3D_t pose_translated = input_data[i].gps_pose;

	pose_translated.position.x -= input_data[0].gps_pose.position.x;
	pose_translated.position.y -= input_data[0].gps_pose.position.y;

	Eigen::Matrix<double, 4, 4> source = transform_carmen_pose_to_pcl_pose(&pose_translated);

	pose_translated = input_data[j].gps_pose;

	pose_translated.position.x -= input_data[0].gps_pose.position.x;
	pose_translated.position.y -= input_data[0].gps_pose.position.y;

	Eigen::Matrix<double, 4, 4> target = transform_carmen_pose_to_pcl_pose(&pose_translated);

	pcl::transformPointCloud(*source_pointcloud_leafed, *source_pointcloud_world, (target.inverse() * source).cast<float>());

	if (perform_icp(source_pointcloud_world, target_pointcloud_leafed, &measured_pose_out))
	{
		carmen_pose_3D_t transform = get_loop_closure_transform(i, j, measured_pose_out);

		Eigen::Matrix<double, 4, 4> T = (measured_pose_out * target.inverse() * source).inverse();

		transform_pcl_pose_to_carmen_pose(T, &transform);

		fprintf(output_file, "%d %d %lf %lf %lf\n", i, j,
			transform.position.x, transform.position.y,
			transform.orientation.yaw
		);

		fflush(output_file);
	}
	else
		fprintf(stderr, "Not converged!\n");

	source_pointcloud->clear();
	target_pointcloud->clear();

	source_pointcloud_world->clear();
	target_pointcloud_world->clear();

	source_pointcloud_leafed->clear();
	target_pointcloud_leafed->clear();

}


void
process_data(double dist_for_detecting_loop_closure, double time_difference_for_detecting_loop_closure)
{
	uint i, j;
	double dist;
	double delta_x;
	double delta_y;
	double delta_t;

	int n = 0;
	double acc = 0;
	double elapsedTime;
	timeval t1, t2;
	double min_dist;
	int loop_id;

	int num_loops_found = 0;

	for (i = 0; i < input_data.size(); i++)
	{
		loop_id = -1;
		min_dist = DBL_MAX;

		for (j = (i + 1); j < input_data.size(); j++)
		{
			delta_x = input_data[i].gps_pose.position.x - input_data[j].gps_pose.position.x;
			delta_y = input_data[i].gps_pose.position.y - input_data[j].gps_pose.position.y;
			delta_t = input_data[i].timestamp - input_data[j].timestamp;

			dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

			if ((dist < min_dist) && (fabs(delta_t) > time_difference_for_detecting_loop_closure)) // Tempo e distancia para detectar fechamento de loop. Alberto: TODO - passar o tempo como parametro.
			{
				min_dist = dist;
				loop_id = j;
			}
		}

		if (min_dist < dist_for_detecting_loop_closure)
		{
			fprintf(stderr, "Pose %d de %d: %lf ", i, (int) input_data.size(), 100.0 * (double) i / (double) input_data.size());
			gettimeofday(&t1, NULL);

			num_loops_found++;
			add_icp_restriction(i, loop_id);

			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec);
			elapsedTime += (t2.tv_usec - t1.tv_usec) / (1000.0 * 1000.0);   // us to s

			n += 1.0;
			acc += elapsedTime;

			fprintf(stderr, " time step: %lf mean: %lf\n", elapsedTime, acc / n);
		}
	}

	printf("num loops found: %d\n", num_loops_found);
}


void
initialize_global_variables(char **argv)
{
	velodyne_storage_dir = argv[2];
	output_filename = argv[3];

	output_file = fopen(output_filename, "w");

	if (output_file == NULL)
		exit(printf("Unable to open file '%s'", output_filename));
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

	if (argc >= 5)
		dist_for_detecting_loop_closure = atof(argv[4]);

	if (argc >= 6)
		time_difference_for_detecting_loop_closure = atof(argv[5]);

	initialize_global_variables(argv);
	initialize_icp();
	read_data(input_file);
	process_data(dist_for_detecting_loop_closure, time_difference_for_detecting_loop_closure);
	clean_data();

	printf("Pressione crtl+c para terminar.\n");
	fflush(stdout);
	getchar();

	return 0;
}


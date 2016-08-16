#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>

#include <tf.h>
#include <vector>
#include <carmen/carmen.h>

using namespace std;
using namespace tf;


typedef struct
{
	carmen_pose_3D_t odometry_pose;
	carmen_pose_3D_t gps_pose;
	double timestamp;
}Line;

char *velodyne_storage_dir = "/media/OS/Users/Filipe/Desktop/velodyne_log_voltadaufes-20130916-tiago/";
//char *velodyne_storage_dir = "/media/OS/Users/Filipe/Desktop/velodyne_log_voltadaufes-20140226";
pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> gicp;
vector<Line> input_data;


Line
read_line(FILE *f)
{
	Line l;

	memset(&l.odometry_pose, 0, sizeof(l.odometry_pose));
	memset(&l.gps_pose, 0, sizeof(l.gps_pose));

	fscanf(f, "%lf %lf %lf %lf %lf %lf %lf\n",
		&(l.odometry_pose.position.x), &(l.odometry_pose.position.y), &(l.odometry_pose.orientation.yaw),
		&(l.gps_pose.position.x), &(l.gps_pose.position.y), &(l.gps_pose.orientation.yaw),
		&l.timestamp
	);

	return l;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr
GridFiltering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputPointCloud, double size)
{
       pcl::VoxelGrid<pcl::PointXYZRGB> grid;
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

       grid.setLeafSize(size, size, size);
       grid.setInputCloud(inputPointCloud);
       grid.filter(*outputPointCloud);

       return outputPointCloud;
}


Eigen::Matrix<double, 4, 4>
transform_carmen_pose_to_pcl_pose(carmen_pose_3D_t *carmen_pose)
{
	Eigen::Matrix<double, 4, 4> pcl_pose;

	Quaternion quat(carmen_pose->orientation.yaw,
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
save_clouds_for_debug(pcl::PointCloud<pcl::PointXYZRGB> source_pointcloud, pcl::PointCloud<pcl::PointXYZRGB> target_pointcloud, pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud_transformed, pcl::PointCloud<pcl::PointXYZRGB> source_pointcloud2)
{
	static int id_icp_call = 1;
	char name[1024];

	if (id_icp_call % 1000 == 0)
	{
		sprintf(name, "%s/%05d_source.ply", velodyne_storage_dir, id_icp_call);
		pcl::io::savePLYFile(name, source_pointcloud2);
		sprintf(name, "%s/%05d_guess.ply", velodyne_storage_dir, id_icp_call);
		pcl::io::savePLYFile(name, source_pointcloud);
		sprintf(name, "%s/%05d_target.ply", velodyne_storage_dir, id_icp_call);
		pcl::io::savePLYFile(name, target_pointcloud);
		sprintf(name, "%s/%05d_result.ply", velodyne_storage_dir, id_icp_call);
		pcl::io::savePLYFile(name, out_pcl_pointcloud_transformed);
		printf("Saved: %s\n", name);
	}

	id_icp_call++;
}


int
perform_icp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pointcloud, Eigen::Matrix<double, 4, 4> *correction, pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud2)
{
	pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud;
	pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud_transformed;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_filtered = GridFiltering(source_pointcloud, 0.2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_filtered = GridFiltering(target_pointcloud, 0.2);

	gicp.setInputCloud(source_filtered);
	gicp.setInputTarget(target_filtered);
	gicp.align(out_pcl_pointcloud);
	(*correction) = gicp.getFinalTransformation().cast<double>();

	pcl::transformPointCloud(out_pcl_pointcloud, out_pcl_pointcloud_transformed, (*correction));
	//save_clouds_for_debug(*source_pointcloud, *target_pointcloud, out_pcl_pointcloud_transformed, *source_pointcloud2);

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
save_restriction(Line l, carmen_pose_3D_t icp_pose)
{
	printf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
		l.odometry_pose.position.x, l.odometry_pose.position.y, l.odometry_pose.orientation.yaw,
		l.gps_pose.position.x, l.gps_pose.position.y, l.gps_pose.orientation.yaw,
		icp_pose.position.x, icp_pose.position.y, icp_pose.orientation.yaw,
		l.timestamp
	);
}


void
initialize_icp()
{
	// gicp.setMaximumIterations(2000);
	gicp.setMaximumIterations(100);
	gicp.setTransformationEpsilon(1e-8);
	gicp.setRotationEpsilon(1e-8);
	// gicp.setMaxCorrespondenceDistance(1.0);
	gicp.setMaxCorrespondenceDistance(2.0);
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
process_data()
{
	uint i;
	char filename_t0[1024];
	char filename_t1[1024];
	carmen_pose_3D_t measured_pose_out;
	Eigen::Matrix<double, 4, 4> correction, T;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_pointcloud_with_guess = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);

	// first pose is set to zero
	memset(&measured_pose_out, 0, sizeof(measured_pose_out));
	save_restriction(input_data[0], measured_pose_out);

	for (i = 0; i < (input_data.size() - 1); i++)
	{
		fprintf(stderr, "%d de %d: percentage: %.2lf\n", i + 1, (int) input_data.size(), 100.0 * ((double) (i + 1) / (double) (input_data.size())));

		sprintf(filename_t0, "%s/%lf.ply", velodyne_storage_dir, input_data[i].timestamp);
		sprintf(filename_t1, "%s/%lf.ply", velodyne_storage_dir, input_data[i + 1].timestamp);

		load_pointcloud_from_file(filename_t0, target_pointcloud);
		load_pointcloud_from_file(filename_t1, source_pointcloud);

		if (source_pointcloud->size() <= 0)
			exit(printf("Error: Source empty: %s\n", filename_t0));

		if (target_pointcloud->size() <= 0)
			exit(printf("Error: Target empty: %s\n", filename_t1));

		// odometry guess
		Eigen::Matrix<double, 4, 4> pose_target = transform_carmen_pose_to_pcl_pose(&(input_data[i].odometry_pose));
		Eigen::Matrix<double, 4, 4> pose_source = transform_carmen_pose_to_pcl_pose(&(input_data[i + 1].odometry_pose));
		Eigen::Matrix<double, 4, 4> guess = pose_target.inverse() * pose_source;

		// gps guess
//		carmen_pose_3D_t gps_pose_i0 = input_data[i].gps_pose;
//		carmen_pose_3D_t gps_pose_i1 = input_data[i + 1].gps_pose;
//
//		gps_pose_i0.position.x -= input_data[0].gps_pose.position.x;
//		gps_pose_i0.position.y -= input_data[0].gps_pose.position.y;
//
//		gps_pose_i1.position.x -= input_data[0].gps_pose.position.x;
//		gps_pose_i1.position.y -= input_data[0].gps_pose.position.y;
//
//		Eigen::Matrix<double, 4, 4> pose_target = transform_carmen_pose_to_pcl_pose(&gps_pose_i0);
//		Eigen::Matrix<double, 4, 4> pose_source = transform_carmen_pose_to_pcl_pose(&gps_pose_i1);
//		Eigen::Matrix<double, 4, 4> guess = pose_target.inverse() * pose_source;

		pcl::transformPointCloud(*source_pointcloud, *source_pointcloud_with_guess, guess);

		if (perform_icp(source_pointcloud_with_guess, target_pointcloud, &correction, source_pointcloud))
		{
			// T = (pose_target.inverse() * correction * guess * pose_source);
			T = (correction * guess);
			transform_pcl_pose_to_carmen_pose(T, &measured_pose_out);
			save_restriction(input_data[i + 1], measured_pose_out);
		}
		else
			fprintf(stderr, "%d: Not converged!\n", i);

		source_pointcloud_with_guess->clear();
	}
}


int
main(int argc, char **argv)
{
	if (argc < 2)
	{
		printf("Use %s <input-file> <output-loops-file>\n", argv[0]);
		exit(1);
	}

	char *input_file = argv[1];
//	char *output_file = argv[2];

	//FILE *g = fopen(output_file, "w");


//	if (g == NULL)
//		exit(printf("File '%s' could not be open\n", output_file));

	initialize_icp();
	read_data(input_file);
	process_data();

//	fclose(g);

	return 0;
}


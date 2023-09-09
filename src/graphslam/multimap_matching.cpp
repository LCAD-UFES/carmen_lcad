#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/registration/gicp.h>
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
	carmen_pose_3D_t pose;
	double timestamp;
}Line;

typedef struct
{
	int i, j;
	carmen_pose_3D_t transform;
}RegistrationRestriction;

typedef pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> ColorGICP;

ColorGICP gicp;
vector<Line> input_data_1;
vector<Line> input_data_2;
vector<RegistrationRestriction> loops;

char *input_file_1;
char *input_file_2;
char *out_loops_filename;
char *velodyne_storage_dir_1;
char *velodyne_storage_dir_2;

FILE *out_loops_file;

Line
read_line(FILE *f)
{
	Line l;

	memset(&l.pose, 0, sizeof(l.pose));

	fscanf(f, "%lf %lf %lf %lf\n",
		&(l.pose.position.x), &(l.pose.position.y), &(l.pose.orientation.yaw),
		&l.timestamp
	);

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
save_clouds_for_debug(pcl::PointCloud<pcl::PointXYZRGB> source_pointcloud, pcl::PointCloud<pcl::PointXYZRGB> target_pointcloud, pcl::PointCloud<pcl::PointXYZRGB> out_pcl_pointcloud_transformed, char *velodyne_storage_dir)
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
	// save_clouds_for_debug(*source_pointcloud, *target_pointcloud, out_pcl_pointcloud_transformed, "velodyne");

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
	gicp.setTransformationEpsilon(1e-5);
	gicp.setRotationEpsilon(1e-5);
	gicp.setMaxCorrespondenceDistance(15.0);
}


vector<Line>
read_data_from_file(char *input_file)
{
	vector<Line> lines;
	FILE *f = fopen(input_file, "r");

	if (f == NULL)
		exit(printf("File '%s' could not be open\n", input_file));

	while(!feof(f))
	{
		Line l = read_line(f);
		lines.push_back(l);
	}

	fclose(f);
	return lines;
}


void
read_data()
{
	input_data_1 = read_data_from_file(input_file_1);
	input_data_2 = read_data_from_file(input_file_2);
}


void
project_to_world(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_world, int i, vector<Line> data, carmen_pose_3D_t first_pose)
{
	carmen_pose_3D_t pose_translated = data[i].pose;

	pose_translated.position.x -= first_pose.position.x;
	pose_translated.position.y -= first_pose.position.y;

	Eigen::Matrix<double, 4, 4> transform = transform_carmen_pose_to_pcl_pose(&pose_translated);
	pcl::transformPointCloud(*pointcloud, *pointcloud_world, transform);
}


carmen_pose_3D_t
get_loop_closure_transform(int i, int j, Eigen::Matrix<double, 4, 4> correction, vector<Line> data_1, vector<Line> data_2, carmen_pose_3D_t first_pose)
{
	carmen_pose_3D_t posei_translated = data_1[i].pose;
	carmen_pose_3D_t posej_translated = data_2[j].pose;
	carmen_pose_3D_t transform;

	posei_translated.position.x -= first_pose.position.x;
	posei_translated.position.y -= first_pose.position.y;

	posej_translated.position.x -= first_pose.position.x;
	posej_translated.position.y -= first_pose.position.y;

	Eigen::Matrix<double, 4, 4> T;
	Eigen::Matrix<double, 4, 4> source = transform_carmen_pose_to_pcl_pose(&posei_translated);
	Eigen::Matrix<double, 4, 4> target = transform_carmen_pose_to_pcl_pose(&posej_translated);

	T = (source.inverse() * correction.inverse() * target);
	transform_pcl_pose_to_carmen_pose(T, &transform);

	return transform;
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
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pointcloud_world;

	if (is_first)
	{
		source_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		target_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		source_pointcloud_world = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
		target_pointcloud_world = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);

		is_first = 0;
	}

	sprintf(filename_t0, "%s/%lf.ply", velodyne_storage_dir_1, input_data_1[i].timestamp);
	sprintf(filename_t1, "%s/%lf.ply", velodyne_storage_dir_2, input_data_2[j].timestamp);

	load_pointcloud_from_file(filename_t0, source_pointcloud);
	load_pointcloud_from_file(filename_t1, target_pointcloud);

	if (source_pointcloud->size() <= 0)
		exit(printf("Error: SourceCloud empty: %s\n", filename_t0));

	if (target_pointcloud->size() <= 0)
		exit(printf("Error: TargetCloud empty: %s\n", filename_t1));

	/**
	 * NOTA: Nas funcoes abaixo a first_pose (ultimo parametro) deve ser do mesmo log para que as nuvens
	 * de pontos fiquem na mesma referencia. Por isso eu coloquei input_data_1[0].pose nas duas. O mesmo
	 * acontece na funcao get_loop_closure_transform.
	 */
	project_to_world(source_pointcloud, source_pointcloud_world, i, input_data_1, input_data_1[0].pose);
	project_to_world(target_pointcloud, target_pointcloud_world, j, input_data_2, input_data_1[0].pose);

	if (perform_icp(source_pointcloud_world, target_pointcloud_world, &measured_pose_out))
	{
		carmen_pose_3D_t transform = get_loop_closure_transform(i, j, measured_pose_out, input_data_1, input_data_2, input_data_1[0].pose);

		fprintf(out_loops_file, "%d %d %lf %lf %lf\n", i, j,
			transform.position.x, transform.position.y,
			transform.orientation.yaw
		);

		fflush(out_loops_file);
	}
	else
		fprintf(stderr, "Not converged!\n");

	source_pointcloud->clear();
	target_pointcloud->clear();

	source_pointcloud_world->clear();
	target_pointcloud_world->clear();
}


void
process_data()
{
	uint i, j;
	double dist;
	double delta_x;
	double delta_y;

	int n = 0;
	double acc = 0;
	double elapsedTime;
	timeval t1, t2;
	double min_dist;
	int loop_id;

	int num_loops_found = 0;

	for (i = 0; i < input_data_1.size(); i++)
	{
		loop_id = -1;
		min_dist = DBL_MAX;

		/*********************************************************************
		 * NOTA: Se o loop abaixo estiver pesado, pode ser usada uma kd-tree para
		 * buscar os pontos mais proximos.
		 *********************************************************************/

		for (j = 0; j < input_data_2.size(); j++)
		{
			delta_x = input_data_1[i].pose.position.x - input_data_2[j].pose.position.x;
			delta_y = input_data_1[i].pose.position.y - input_data_2[j].pose.position.y;

			dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

			if (dist < min_dist)
			{
				min_dist = dist;
				loop_id = j;
			}
		}

		if (min_dist < 30.0)
		{
			fprintf(stderr, "Pose %d de %d: %lf ", i, (int) input_data_1.size(), 100.0 * (double) i / (double) input_data_1.size());
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
	input_file_1 = argv[1];
	input_file_2 = argv[3];

	out_loops_filename = argv[5];

	velodyne_storage_dir_1 = argv[2];
	velodyne_storage_dir_2 = argv[4];

	out_loops_file = fopen(out_loops_filename, "w");

	if (out_loops_file == NULL)
		exit(printf("Unable to open file '%s'", out_loops_filename));
}


void
clean_data()
{
	fclose(out_loops_file);
}


int
main(int argc, char **argv)
{
	/***
	 * NOTA: Existiam duas opcoes para fazer esse programa:
	 *
	 * 1. Assumir que as poses ja foram otimizadas, colocar as
	 * restricoes de uma pose para a proxima com uma covariancia
	 * muito baixa e colocar as restricoes de encaixe das poses.
	 *
	 * 2. Colocar todas as restricoes (gps/imu, odometria e fechamento de loop)
	 * e as novas restricoes de encaixe das poses e reotimizar tudo.
	 *
	 * A segunda opcao eh mais correta, mas a primeira eh muito mais simples. Eu
	 * escolhi fazer a primeira. Se um dia esse programa nao funcionar, pode ser
	 * que a segunda tenha que ser implementada.
	 */

	if (argc < 6)
	{
		printf("Use %s <poses-file-1> <velodyne-dir-1> <poses-file-2> <velodyne-dir-2> <loops-out.txt>\n", argv[0]);
		exit(1);
	}

	initialize_global_variables(argv);
	initialize_icp();
	read_data();
	process_data();
	clean_data();

	printf("Tecle ctrl+c para terminar.\n");
	getchar();
	
	return 0;
}


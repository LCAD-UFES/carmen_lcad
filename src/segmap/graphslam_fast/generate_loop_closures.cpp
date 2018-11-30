

#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cfloat>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <carmen/carmen.h>


using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace cv;


double velodyne_vertical_angles[32] =
{
		-30.6700000, -29.3300000, -28.0000000, -26.6700000, -25.3300000, -24.0000000, -22.6700000, -21.3300000,
		-20.0000000, -18.6700000, -17.3300000, -16.0000000, -14.6700000, -13.3300000, -12.0000000, -10.6700000,
		-9.3299999, -8.0000000, -6.6700001, -5.3299999, -4.0000000, -2.6700001, -1.3300000, 0.0000000, 1.3300000,
		2.6700001, 4.0000000, 5.3299999, 6.6700001, 8.0000000, 9.3299999, 10.6700000
};

int velodyne_ray_order[32] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};

const double TIME_SPENT_IN_EACH_SCAN = 0.000046091445;
const double distance_between_rear_wheels = 1.535;
const double distance_between_front_and_rear_axles = 2.625;
const double distance_between_front_car_and_front_wheels = 0.85;
const double distance_between_rear_car_and_rear_wheels = 0.96;
const double car_length = (distance_between_front_and_rear_axles +
						distance_between_rear_car_and_rear_wheels +
						distance_between_front_car_and_front_wheels);
const double car_width = distance_between_rear_wheels;


class Data
{
public:
	char cloud_path[128];
	int n_rays;
	double cloud_time;

	char image_path[128];
	int w, h, size, n;
	double image_time;

	double x, y, quality, gps_time;
	double angle, angle_quality, gps2_time;

	double v, phi, ack_time;
};


void
load_data(char *name, vector<Data> &lines)
{
	FILE *f = fopen(name, "r");
	char dummy[128];
	int idummy;

	while (!feof(f))
	{
		Data d;

		char c = fgetc(f);
		if (c != 'V')
			continue;

		fscanf(f, "\n%s %s %d %lf ",
				dummy, d.cloud_path, &d.n_rays, &d.cloud_time);

		fscanf(f, " %s %s %d %d %d %d %lf ",
				dummy, d.image_path, &d.w, &d.h, &d.size, &d.n, &d.image_time);

		fscanf(f, " %s %d %lf %lf %lf %lf ",
			dummy, &idummy, &d.x, &d.y, &d.quality, &d.gps_time);

		fscanf(f, " %s %d %lf %lf %lf ",
			dummy,  &idummy, &d.angle, &d.angle_quality, &d.gps2_time);

		fscanf(f, " %s %lf %lf %lf\n",
			dummy, &d.v, &d.phi, &d.ack_time);

		lines.push_back(d);
	}

	fclose(f);
}


void
find_loop_closure_poses(vector<Data> &lines, vector<pair<int, int>> &loop_closure_indices)
{
	for (uint i = 0; i < lines.size(); i++)
	{
		int min_id = -1;
		double min_dist = DBL_MAX;

		for (uint j = i + 1; j < lines.size(); j++)
		{
			double delta_x = lines[i].x - lines[j].x;
			double delta_y = lines[i].y - lines[j].y;
			double delta_t = lines[i].gps_time - lines[j].gps_time;

			double dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));

			if ((dist < min_dist) && (fabs(delta_t) > 300.0))
			{
				min_dist = dist;
				min_id = j;
			}
		}

		if ((min_id != -1) && (min_dist < 10.))
			loop_closure_indices.push_back(pair<int, int>(i, min_id));
	}

	printf("Num loop closures: %ld\n", loop_closure_indices.size());
}


PointXYZRGB
compute_point_from_velodyne(double v_angle, double h_angle, double radius, unsigned char intensity, unsigned char r, unsigned char g, unsigned char b)
{
    // build a new point
    PointXYZRGB point;

	double cos_rot_angle = cos(h_angle);
	double sin_rot_angle = sin(h_angle);

	double cos_vert_angle = cos(v_angle);
	double sin_vert_angle = sin(v_angle);

	double xy_distance = radius * cos_vert_angle;

	point.x = (xy_distance * cos_rot_angle);
	point.y = (xy_distance * sin_rot_angle);
	point.z = (radius * sin_vert_angle);

	point.r = r;
    point.g = g;
    point.b = b;

    return point;
}


void
load_pointcloud(Data &d, PointCloud<PointXYZRGB>::Ptr cloud, unsigned char r, unsigned char g, unsigned char b)
{
	FILE *f = fopen(d.cloud_path, "rb");

	double th, ds;
	double h_angle, v_angle;
	unsigned short distances[32];
	unsigned char intensities[32];
	double range;

	cloud->clear();

	th = 0;

	for(int i = 0; i < d.n_rays; i++)
	{
		fread(&h_angle, sizeof(double), 1, f);
	    fread(distances, sizeof(unsigned short), 32, f);
	    fread(intensities, sizeof(unsigned char), 32, f);

	    h_angle = M_PI * h_angle / 180.;

	    for (int j = 0; j < 32; j++)
	    {
	    	range = (double) distances[velodyne_ray_order[j]] / 500.;
	    	v_angle = velodyne_vertical_angles[j];
	    	v_angle = M_PI * v_angle / 180.;

	    	PointXYZRGB point = compute_point_from_velodyne(v_angle, h_angle, range, intensities[velodyne_ray_order[j]], r, g, b);

	    	if (range > 70.  || (fabs(point.x) < 4. && fabs(point.y) < 2.)) // || point.z < -1.5 || point.z > 1.5)
	    		continue;

	    	ds = d.v * (i * TIME_SPENT_IN_EACH_SCAN);
	    	point.x += ds * cos(th);
	    	point.y += ds * sin(th);
	    	th = carmen_normalize_theta(ds * (tan(d.phi) / distance_between_front_and_rear_axles));

	    	cloud->push_back(point);
	    }
	}

    fclose(f);
}


Matrix<double, 4, 4>
create_transformation_matrix(double x, double y, double th)
{
	Matrix<double, 4, 4> m;

	m << cos(th), -sin(th), 0, x,
			sin(th), cos(th), 0, y,
			0, 0, 1, 0,
			0, 0, 0, 1;

	return m;
}


void
run_gicp(vector<Data> &lines, vector<pair<int, int>> &loop_closure_indices, vector<Matrix<double, 4, 4>> &transforms, vector<bool> &converged)
{
	int i;

	GeneralizedIterativeClosestPoint<PointXYZRGB, PointXYZRGB> gicp;
	gicp.setMaximumIterations(2000);
	gicp.setTransformationEpsilon(1e-3);
	gicp.setMaxCorrespondenceDistance(15.0);

	//visualization::PCLVisualizer viewer("Cloud Viewer");
	//viewer.setBackgroundColor(.5, .5, .5);
	//viewer.removeAllPointClouds();

	#pragma omp parallel for default(none) shared(lines, transforms, converged, loop_closure_indices) private(i, gicp)
	for (i = 0; i < loop_closure_indices.size(); i++)
	{
		printf("%d of %d\n", i, loop_closure_indices.size());

		PointCloud<PointXYZRGB>::Ptr source(new PointCloud<PointXYZRGB>), target(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr source_world(new PointCloud<PointXYZRGB>), target_world(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr source_leafed(new PointCloud<PointXYZRGB>), target_leafed(new PointCloud<PointXYZRGB>);
		PointCloud<PointXYZRGB>::Ptr output(new PointCloud<PointXYZRGB>), output_transformed(new PointCloud<PointXYZRGB>);

		const double leaf_size = 0.25;
	    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
	    grid.setLeafSize(leaf_size, leaf_size, leaf_size);

		Data source_data = lines[loop_closure_indices[i].first];
		Data target_data = lines[loop_closure_indices[i].second];

		load_pointcloud(source_data, source, 255, 0, 0);
		load_pointcloud(target_data, target, 0, 0, 255);

	    grid.setInputCloud(source);
	    grid.filter(*source_leafed);

	    grid.setInputCloud(target);
	    grid.filter(*target_leafed);

	    Matrix<double, 4, 4> source_t = create_transformation_matrix(source_data.x - target_data.x,
	    		source_data.y - target_data.y, source_data.angle);
	    Matrix<double, 4, 4> target_t = create_transformation_matrix(0., 0., target_data.angle);
	    Matrix<double, 4, 4> gicp_correction;

	    pcl::transformPointCloud(*source_leafed, *source_world, (target_t.inverse() * source_t).cast<float>());

	    gicp.setInputCloud(source_world);
	    gicp.setInputTarget(target_leafed);
	    gicp.align(*output);
	    gicp_correction = gicp.getFinalTransformation().cast<double>();

	    //output_transformed = output;
	    //output_transformed = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>(*target_leafed));
	    // pcl::transformPointCloud(*source_leafed, *output_transformed, (gicp_correction * target_t.inverse() * source_t).cast<float>());
	    //	// save_clouds_for_debug(*source_pointcloud, *target_pointcloud, out_pcl_pointcloud_transformed);
	    //for (int k = 0; k < output_transformed->size(); k++)
	    //{
	    	//output_transformed->at(k).g = 255;
	    //}

		//viewer.removeAllPointClouds();
		//viewer.addPointCloud(source_world, "source");
		//viewer.addPointCloud(target_leafed, "target");
		//viewer.addPointCloud(output_transformed, "transformed");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed");
        //
		//char c = ' ';
		//while (c != 'n')
		//{
		//	imshow("map", Mat::zeros(300, 300, CV_8UC3));
		//	viewer.spinOnce();
		//	c = waitKey(5);
		//}

		transforms[i] = (gicp_correction * target_t.inverse() * source_t).inverse();
		converged[i] = gicp.hasConverged();

//		source_pointcloud_leafed = LeafSize(source_pointcloud, 0.2);
//		target_pointcloud_leafed = LeafSize(target_pointcloud, 0.2);
//
//		carmen_pose_3D_t pose_translated = input_data[i].gps_pose;
//
//		pose_translated.position.x -= input_data[0].gps_pose.position.x;
//		pose_translated.position.y -= input_data[0].gps_pose.position.y;
//
//		Eigen::Matrix<double, 4, 4> source = transform_carmen_pose_to_pcl_pose(&pose_translated);
//
//		pose_translated = input_data[j].gps_pose;
//
//		pose_translated.position.x -= input_data[0].gps_pose.position.x;
//		pose_translated.position.y -= input_data[0].gps_pose.position.y;
//
//		Eigen::Matrix<double, 4, 4> target = transform_carmen_pose_to_pcl_pose(&pose_translated);
//
//		pcl::transformPointCloud(*source_pointcloud_leafed, *source_pointcloud_world, (target.inverse() * source).cast<float>());
//
//		if (perform_icp(source_pointcloud_world, target_pointcloud_leafed, &measured_pose_out))
//		{
//			carmen_pose_3D_t transform = get_loop_closure_transform(i, j, measured_pose_out);
//			Eigen::Matrix<double, 4, 4> T = (measured_pose_out * target.inverse() * source).inverse();
//			transform_pcl_pose_to_carmen_pose(T, &transform);
//
//			// add to output
//		}
	}
}


double theta_from_matrix(Matrix<double, 4, 4> &m)
{
	// extract rotation matrix
	static Matrix<double, 3, 3> R;
	R << m(0, 0), m(0, 1), m(0, 2),
			m(1, 0), m(1, 1), m(1, 2),
			m(2, 0), m(2, 1), m(2, 2);

	// Important:
	// Extracting the yaw component from the rotation matrix is not
	// the right wayt of computing theta. Note that a rotation of yaw=pitch=roll=0
	// is equivalent to a rotation of yaw=pitch=roll=3.14 (in this order), but
	// yaw=0. is the opposite of yaw=3.14.
	// Matrix<double, 3, 1> ypr = R.eulerAngles(2, 1, 0);

	// Here I'm using the ad-hoc approach of rotating an unitary vector
	// and computing theta using the x and y coordinates. TODO: find a principled approach.
	static Matrix<double, 3, 1> unit;
	unit << 1, 0, 0;
	unit = R * unit;

	return atan2(unit(1, 0), unit(0, 0));
}


void
write_output(char *filename, vector<pair<int, int>> &loop_closure_indices, vector<Matrix<double, 4, 4>> &transforms, vector<bool> &converged)
{
	FILE *f = fopen(filename, "w");

	for (unsigned int i = 0; i < loop_closure_indices.size(); i++)
	{
		double x, y, th;

		x = transforms[i](0, 3) / transforms[i](3, 3);
		y = transforms[i](1, 3) / transforms[i](3, 3);
		th = theta_from_matrix(transforms[i]);

		fprintf(f, "%d %d %d %lf %lf %lf\n",
				loop_closure_indices[i].first, loop_closure_indices[i].second,
				(int) converged[i], x, y, th);
	}

	fclose(f);
}


int
main(int argc, char **argv)
{
	if (argc < 3)
	{
		printf("\nError: Use %s <sync_file> <output_file>\n\n", argv[0]);
		exit(0);
	}

	vector<Data> lines;
	vector<pair<int, int>> loop_closure_indices;

	load_data(argv[1], lines);
	find_loop_closure_poses(lines, loop_closure_indices);

	if (loop_closure_indices.size() > 0)
	{
		vector<Matrix<double, 4, 4>> transforms(loop_closure_indices.size());
		vector<bool> converged(loop_closure_indices.size());
		run_gicp(lines, loop_closure_indices, transforms, converged);
		write_output(argv[2], loop_closure_indices, transforms, converged);
	}

	return 0;
}


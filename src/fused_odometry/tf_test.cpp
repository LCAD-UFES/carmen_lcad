#include <carmen/carmen.h>

#include <tf.h>
#include <stdio.h>

static carmen_orientation_3D_t get_carmen_orientation_from_tf_transform(tf::Transform transform)
{
	carmen_orientation_3D_t orientation;
	tf::Matrix3x3(transform.getRotation()).getEulerYPR(orientation.yaw, orientation.pitch, orientation.roll);
	
	return orientation;
}

static void print_transform(tf::Transform transform)
{	
	carmen_orientation_3D_t orientation = get_carmen_orientation_from_tf_transform(transform);
	printf("y:% lf p:% lf r:% lf\n", orientation.yaw, orientation.pitch, orientation.roll);
	printf("x:% lf y:% lf z:% lf\n", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
	printf("\n");
}

static void test_transforms()
{
	double PI = carmen_degrees_to_radians(180);

	tf::Transform t1;
	t1.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
	t1.setRotation(tf::Quaternion(PI/2, 0.0, 0.0));

	tf::Transform t2;
	t2.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
	t2.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

	tf::Transform t3 = t1*t2;

	print_transform(t1);
	print_transform(t2);
	print_transform(t3);
}

int main()
{
	// See: http://www.ros.org/wiki/tf

	tf::Transform world_to_car;
	world_to_car.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	world_to_car.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

	tf::Transform car_to_gps;
	car_to_gps.setOrigin(tf::Vector3(-1.0, 0.0, 0.0));
	car_to_gps.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

	tf::Transform car_to_xsens;
	car_to_xsens.setOrigin(tf::Vector3(0.5, 0.2, 0.0));
	car_to_xsens.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

	tf::Transform car_to_camera;
	car_to_camera.setOrigin(tf::Vector3(0.5, -0.2, 0.0));
	car_to_camera.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

	tf::Time::init();
	tf::Transformer transformer(false);

	transformer.setTransform(tf::StampedTransform(world_to_car, tf::Time(0), "/world", "/car"));
	transformer.setTransform(tf::StampedTransform(car_to_gps, tf::Time(0), "/car", "/gps"));
	transformer.setTransform(tf::StampedTransform(car_to_xsens, tf::Time(0), "/car", "/xsens"));
	transformer.setTransform(tf::StampedTransform(car_to_camera, tf::Time(0), "/car", "/camera"));

	// ----------------------------- //

	//printf("Initial Poses:\n");

	tf::StampedTransform camera_to_world;
	transformer.lookupTransform("/world", "/camera", tf::Time(0), camera_to_world);
	//printf("Camera pose with respect to world  : x: % 6.2f, y: % 6.2f, z: % 6.2f\n", camera_to_world.getOrigin().x(), camera_to_world.getOrigin().y(), camera_to_world.getOrigin().z());

	for (double n = 0.0; n < 100.0 * 100.0; n += 1.0)
	{
		for (double z = 0.0; z < 100000.0; z += 1.0)
		{
			world_to_car.setOrigin(tf::Vector3(z, 0.0, 0.0));
			world_to_car.setRotation(tf::Quaternion(0.0, 0.0, 0.0));

			transformer.setTransform(tf::StampedTransform(world_to_car, tf::Time(0), "/world", "/car"));

			transformer.lookupTransform("/world", "/camera", tf::Time(0), camera_to_world);
			//printf("Camera pose with respect to world  : x: % 6.2f, y: % 6.2f, z: % 6.2f\n", camera_to_world.getOrigin().x(), camera_to_world.getOrigin().y(), camera_to_world.getOrigin().z());
		}
	}

	transformer.lookupTransform("/car", "/gps", tf::Time(0), camera_to_world);
	//printf("Gps pose with respect to car  : x: % 6.2f, y: % 6.2f, z: % 6.2f\n", camera_to_world.getOrigin().x(), camera_to_world.getOrigin().y(), camera_to_world.getOrigin().z());

	tf::Transform world_to_gps;
	world_to_gps.setOrigin(tf::Vector3(-1.0, 0.0, 0.0));
	world_to_gps.setRotation(tf::Quaternion(0.0, 0.0, 0.0));
	transformer.setTransform(tf::StampedTransform(car_to_gps, tf::Time(0), "/world", "/gps"));

	transformer.lookupTransform("/car", "/gps", tf::Time(0), camera_to_world);
	//printf("Gps pose with respect to car  : x: % 6.2f, y: % 6.2f, z: % 6.2f\n", camera_to_world.getOrigin().x(), camera_to_world.getOrigin().y(), camera_to_world.getOrigin().z());


	test_transforms();	

	return 0;
}

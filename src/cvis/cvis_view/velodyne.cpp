#include <velodyne.h>

namespace CVIS {

typedef struct {
	float r, g, b;
}color_table_t;

float sin_rot_table[36000];
float cos_rot_table[36000];
float sin_vert_table[32];
float cos_vert_table[32];

static color_table_t color_table[] = {{0.08, 0.00, 0.98},
									  {0.00, 0.04, 0.98},
									  {0.00, 0.17, 0.98},
									  {0.00, 0.30, 0.98},
									  {0.00, 0.43, 0.98},
									  {0.00, 0.56, 0.98},
									  {0.00, 0.68, 0.98},
									  {0.00, 0.81, 0.98},
									  {0.01, 0.94, 0.98},
									  {0.01, 0.98, 0.89},
									  {0.01, 0.94, 0.98},
									  {0.01, 0.98, 0.77},
									  {0.01, 0.98, 0.64},
									  {0.01, 0.98, 0.52},
									  {0.01, 0.98, 0.39},
									  {0.01, 0.98, 0.26},
									  {0.01, 0.98, 0.14},
									  {0.02, 0.98, 0.02},
									  {0.15, 0.98, 0.02},
									  {0.27, 0.98, 0.02},
									  {0.40, 0.98, 0.02},
									  {0.53, 0.98, 0.02},
									  {0.65, 0.98, 0.02},
									  {0.78, 0.99, 0.02},
									  {0.91, 0.99, 0.02},
									  {0.99, 0.94, 0.02},
									  {0.99, 0.82, 0.02},
									  {0.99, 0.69, 0.02},
									  {0.99, 0.56, 0.03},
									  {0.99, 0.44, 0.03},
									  {0.99, 0.31, 0.03},
									  {0.99, 0.19, 0.03},
									  {0.99, 0.00, 0.00}};

/* vertical correction values extracted from db32.xml from velodyne kit cd-rom (works only with velodyne hdl-32e)*/
static double vertical_correction[32] = { -30.67, -9.3299999, -29.33, -8.0, -28.0, -6.6700001, -26.67, -5.3299999, -25.33, -4.0, -24.0,
		-2.6700001, -22.67, -1.33, -21.33, 0.0, -20.0, 1.33, -18.67, 2.6700001, -17.33, 4.0, -16.0, 5.3299999, -14.67, 6.6700001, -13.33, 8.0,
		-12.0, 9.3299999, -10.67, 10.67 };

	Velodyne::Velodyne(int n)
	{
		this->message = NULL;
		this->number_of_32_laser_shots = n;

		//this->transformer = tf::Transformer(false);

		SetupRotationalAndVerticalTables();
	}

	Velodyne::~Velodyne()
	{

	}

	void Velodyne::PopulatePointCloud(void* message, carmen_fused_odometry_message* car_poses)
	{
		int velodyne_360_shot_position = 0;
		double min_timestamp_diference_between_fusedodometry_and_velodyne_messages = 9999999;
		int index_fc = -1;
		this->message = (carmen_velodyne_partial_scan_message*) message;

		for(int k = 0; k < 100; k++)
		{
			double time_difference = fabs(this->message->timestamp - car_poses[k].timestamp);
			if(time_difference < min_timestamp_diference_between_fusedodometry_and_velodyne_messages)
			{
				min_timestamp_diference_between_fusedodometry_and_velodyne_messages = time_difference;
				index_fc = k;
			}
		}

		tf::Pose best_car_pose;

		best_car_pose.setOrigin(tf::Vector3(car_poses[index_fc].pose.position.x, car_poses[index_fc].pose.position.y, car_poses[index_fc].pose.position.z));			// x, y, z;
		best_car_pose.setRotation(tf::Quaternion(car_poses[index_fc].pose.orientation.yaw, car_poses[index_fc].pose.orientation.pitch, car_poses[index_fc].pose.orientation.roll));		// yaw, pitch, roll

		tf::StampedTransform world_to_car_transform(best_car_pose, tf::Time(0), "/world", "/car");
		this->transformer.setTransform(world_to_car_transform, "world_to_car_transform");

		for(int l = 0; l < this->message->number_of_32_laser_shots; l++)
		{
			velodyne_360_shot_position = (int) floor(this->message->partial_scan[l].angle);
			velodyne360Shots[velodyne_360_shot_position].shots.push_back(this->message->partial_scan[l]);

			if(this->velodyne360Shots[velodyne_360_shot_position].visited == false)
				this->velodyne360Shots[velodyne_360_shot_position].visited = true;

			if(VisitedAllVelodyne360Shots()) //at this point we have a complete 360 velodyne point cloud in velodyne360Shots;
			{
				// get the transformation between the visual odometry coordinate system with respect to the carmen coordinate system.
				transformer.lookupTransform("/world", "/velodyne", tf::Time(0), this->g_world_to_velodyne_transform);

				if(this->pointCloud->vertices != NULL && this->pointCloud->colors !=NULL)
				{
					int pos;
					int shot_size = 0;

					float xy_distance, distance;
					unsigned char intensity;
					float cos_vert_angle, sin_vert_angle, cos_rot_angle, sin_rot_angle;

					for(int p = 0; p < 360; p++)
					{
						for(unsigned int i = 0; i < this->velodyne360Shots[p].shots.size(); i++)
						{
							for(int j = 0; j < 32; j++)
							{
								pos = (shot_size * 32) + (i * 32) + j;

								distance = 0.2 * this->velodyne360Shots[p].shots[i].distance[j];
								intensity = this->velodyne360Shots[p].shots[i].intensity[j];

								if (1)//(distance > 50.0 && distance < 4000.0)
								{
									cos_vert_angle = cos_vert_table[j];
									sin_vert_angle = sin_vert_table[j];
									cos_rot_angle = cos_rot_table[(int)(this->velodyne360Shots[p].shots[i].angle * 100.0)];
									sin_rot_angle = sin_rot_table[(int)(this->velodyne360Shots[p].shots[i].angle * 100.0)];

									xy_distance = distance * cos_vert_angle;

									tf::Transform velodyne_point, velodyne_point_reference_world;
									velodyne_point.setOrigin(tf::Vector3((xy_distance * sin_rot_angle) / 100.0, (xy_distance * cos_rot_angle) / 100.0, (distance * sin_vert_angle) / 100.0));			// x, y, z;
									velodyne_point.setRotation(tf::Quaternion(0.0, 0.0, 0.0));		// yaw, pitch, roll

									velodyne_point_reference_world = this->g_world_to_velodyne_transform * velodyne_point;

									this->pointCloud->vertices[3 * pos] = velodyne_point_reference_world.getOrigin().x(); // (xy_distance * sin_rot_angle) / 100.0;
									this->pointCloud->vertices[3 * pos + 1] = velodyne_point_reference_world.getOrigin().y(); //(xy_distance * cos_rot_angle) / 100.0;
									this->pointCloud->vertices[3 * pos + 2] = velodyne_point_reference_world.getOrigin().z(); //(distance * sin_vert_angle) / 100.0;

//									if(this->pointCloud->vertices[3 * pos + 2] > -3.0)
//									{
//										double z  = this->pointCloud->vertices[3 * pos + 2];
//
//										int color = (int)(((z + 2.0) / 4.0) * 32.0);
//
//										if(color < 0)
//											color = 0;
//										if(color > 31)
//											color  = 31;

										this->pointCloud->colors[3 * pos]     = (double) intensity / 255.0; // color_table[31 - color].r; //r
										this->pointCloud->colors[3 * pos + 1] = (double) intensity / 255.0; // color_table[31 - color].g; //g
										this->pointCloud->colors[3 * pos + 2] = (double) intensity / 255.0; // color_table[31 - color].b; //b
//									}
//									else
//									{
//
//										this->pointCloud->colors[3 * pos] = 0.0;
//										this->pointCloud->colors[3 * pos + 1] = 0.0;
//										this->pointCloud->colors[3 * pos + 2] = 0.0;
//									}
								}
								else
								{
									this->pointCloud->vertices[3 * pos] = 0.0;
									this->pointCloud->vertices[3 * pos + 1] = 0.0;
									this->pointCloud->vertices[3 * pos + 2] = 0.0;

									this->pointCloud->colors[3 * pos] = 0.0;
									this->pointCloud->colors[3 * pos + 1] = 0.0;
									this->pointCloud->colors[3 * pos + 2] = 0.0;
								}
								}
							}

						shot_size += this->velodyne360Shots[p].shots.size();
					}
				}
				else
				{
					printf("ERRO: PointCloud class not initialized!\n");
				}

				InitializeVelodyne360Shots();
			}
		}
	}

	void Velodyne::SetupRotationalAndVerticalTables()
	{
		// Set up cached values for sin and cos of all the possible headings
		for (unsigned short rot_index = 0; rot_index < 36000; ++rot_index)
		{
			float rotation = carmen_degrees_to_radians(0.01 * rot_index);
			cos_rot_table[rot_index] = cosf(rotation);
			sin_rot_table[rot_index] = sinf(rotation);
		}

		for (unsigned short vert_index = 0; vert_index < 32; ++vert_index)
		{
			float vert_radian = carmen_degrees_to_radians(vertical_correction[vert_index]);
			cos_vert_table[vert_index] = cosf(vert_radian);
			sin_vert_table[vert_index] = sinf(vert_radian);
		}
	}

	void Velodyne::InitializeVelodyne360Shots()
	{
		for(int i=0; i < 360; i++)
		{
			this->velodyne360Shots[i].visited = false;
			this->velodyne360Shots[i].shots.clear();
		}
	}

	bool Velodyne::VisitedAllVelodyne360Shots()
	{
		bool result = true;

		for(int i=0; i < 360; i++)
			result &= velodyne360Shots[i].visited;

		return result;
	}

	/* Transforms */

	void Velodyne::InitializeVelodyneTransforms(carmen_vector_3D_t position, carmen_orientation_3D_t orientation)
	{
		// See: http://www.ros.org/wiki/tf
		tf::Transform car_to_velodyne_pose;
		tf::Transform world_to_car_pose;

		tf::Time::init();

		// initial car pose with respect to the world
		world_to_car_pose.setOrigin(tf::Vector3(0.0, 0.0, 0.0));			// x, y, z;
		world_to_car_pose.setRotation(tf::Quaternion(0.0, 0.0, 0.0));		// yaw, pitch, roll

		tf::StampedTransform world_to_car_transform(world_to_car_pose, tf::Time(0), "/world", "/car");
		this->transformer.setTransform(world_to_car_transform, "world_to_car_transform");

		// velodyne pose with respect to the car
		car_to_velodyne_pose.setOrigin(tf::Vector3(position.x, position.y, position.z));					// x, y, z;
		car_to_velodyne_pose.setRotation(tf::Quaternion(orientation.yaw, orientation.pitch, orientation.roll)); 				// yaw, pitch, roll

		tf::StampedTransform car_to_velodyne_transform(car_to_velodyne_pose, tf::Time(0), "/car", "/velodyne");
		this->transformer.setTransform(car_to_velodyne_transform, "car_to_velodyne_transform");
	}
}

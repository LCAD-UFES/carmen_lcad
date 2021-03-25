#include "neural_object_detector2.hpp"
#include <carmen/tf.h>
#include <sys/stat.h>

#define SHOW_DETECTIONS

//bool rectangles_intersects(cv::Point l1, cv::Point r1, cv::Point l2, cv::Point r2);
bool rectangles_intersects(cv::Rect A, cv::Rect B);

using namespace std;

cv::Mat or_image;
int camera;
int camera_side;
int qtd_crops;
double meters_spacement;
char *log_name;
char *crops_tam;
char *groundtruth_path;
char *detection_type;
carmen_camera_parameters camera_parameters;
carmen_pose_3D_t velodyne_pose;
carmen_pose_3D_t bullbar_pose;
carmen_pose_3D_t sick_pose;
carmen_pose_3D_t camera_pose;
carmen_pose_3D_t board_pose;
tf::Transformer transformer;
tf::Transformer transformer_sick;

string str_folder_name;
string str_folder_image_name;
string str_folder_image_name_original;
string str_folder_image_name_slices;
string str_folder_image_name_slices_rddf_filtered;
string str_folder_image_name_slices_rddf_full;
string str_folder_image_name_rddf_full;
string str_folder_image_name_rddf_filtered;
string str_folder_image_name_slices_and_detection;

const unsigned int maxPositions = 50;
carmen_velodyne_partial_scan_message *velodyne_message_arrange;
vector<carmen_velodyne_partial_scan_message> velodyne_vector;

carmen_laser_ldmrs_new_message* sick_laser_message;
carmen_velodyne_partial_scan_message sick_message_arrange;
vector<carmen_laser_ldmrs_new_message> sick_vector;


Detector *darknet;
vector<string> obj_names;


carmen_moving_objects_point_clouds_message moving_objects_point_clouds_message;
carmen_point_t globalpos;
carmen_pose_3D_t pose;

carmen_ackerman_traj_point_t rddf_msg;
bool goal_ready, use_rddf;

carmen_behavior_selector_road_profile_message path_goals_and_annotations_message;

carmen_rddf_annotation_message last_rddf_annotation_message;
carmen_behavior_selector_road_profile_message last_rddf_poses;

bool last_rddf_annotation_message_valid = false;
double last_pitch;

SampleFilter filter2;

carmen_playback_command_message command_of_playback;

#define CAM_DELAY 0.2
#define MAX_POSITIONS 10
// This function find the closest velodyne message with the camera message
carmen_velodyne_partial_scan_message
find_velodyne_most_sync_with_cam(double bumblebee_timestamp)  // TODO is this necessary?
{
	bumblebee_timestamp -= CAM_DELAY;
	carmen_velodyne_partial_scan_message velodyne;
    double minTimestampDiff = DBL_MAX;
    int minTimestampIndex = -1;

    for (unsigned int i = 0; i < velodyne_vector.size(); i++)
    {
        if (fabs(velodyne_vector[i].timestamp - bumblebee_timestamp) < minTimestampDiff)
        {
            minTimestampIndex = i;
            minTimestampDiff = fabs(velodyne_vector[i].timestamp - bumblebee_timestamp);
        }
    }

    velodyne = velodyne_vector[minTimestampIndex];
    return (velodyne);
}


carmen_laser_ldmrs_new_message
find_sick_most_sync_with_cam(double bumblebee_timestamp)  // TODO is this necessary?
{
	bumblebee_timestamp -= CAM_DELAY;
	carmen_laser_ldmrs_new_message sick;
    double minTimestampDiff = DBL_MAX;
    int minTimestampIndex = -1;

    for (unsigned int i = 0; i < sick_vector.size(); i++)
    {
        if (fabs(sick_vector[i].timestamp - bumblebee_timestamp) < minTimestampDiff)
        {
            minTimestampIndex = i;
            minTimestampDiff = fabs(sick_vector[i].timestamp - bumblebee_timestamp);
        }
    }

    sick = sick_vector[minTimestampIndex];
    return (sick);
}


void
build_moving_objects_message(vector<carmen_tracked_cluster_t> clusters)
{

    moving_objects_point_clouds_message.num_point_clouds = clusters.size();
    moving_objects_point_clouds_message.point_clouds = (t_point_cloud_struct *) (malloc(
            moving_objects_point_clouds_message.num_point_clouds * sizeof(t_point_cloud_struct)));


    for (int i = 0; i < moving_objects_point_clouds_message.num_point_clouds; i++) {
        carmen_vector_3D_t box_centroid = compute_centroid(clusters[i].points);
        carmen_vector_3D_t offset;

        // TODO ler do carmen ini
        offset.x = 0.572;
        offset.y = 0.0;
        offset.z = 2.154;

        box_centroid = translate_point(box_centroid, offset);
        box_centroid = rotate_point(box_centroid, globalpos.theta);

        offset.x = globalpos.x;
        offset.y = globalpos.y;
        offset.z = 0.0;

        box_centroid = translate_point(box_centroid, offset);

        moving_objects_point_clouds_message.point_clouds[i].r = 1.0;
        moving_objects_point_clouds_message.point_clouds[i].g = 1.0;
        moving_objects_point_clouds_message.point_clouds[i].b = 0.0;

        moving_objects_point_clouds_message.point_clouds[i].linear_velocity = 0;//clusters[i].linear_velocity;
        moving_objects_point_clouds_message.point_clouds[i].orientation = globalpos.theta;//clusters[i].orientation;

        moving_objects_point_clouds_message.point_clouds[i].object_pose.x = box_centroid.x;
        moving_objects_point_clouds_message.point_clouds[i].object_pose.y = box_centroid.y;
        moving_objects_point_clouds_message.point_clouds[i].object_pose.z = box_centroid.z;

        moving_objects_point_clouds_message.point_clouds[i].height = 1.8;
        moving_objects_point_clouds_message.point_clouds[i].length = 4.5;
        moving_objects_point_clouds_message.point_clouds[i].width = 1.6;

        switch (clusters[i].cluster_type) {

            case carmen_moving_object_type::pedestrian:
                moving_objects_point_clouds_message.point_clouds[i].geometric_model = 0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.height = 1.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.length = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.width = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.red = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.green = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.blue = 0.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.model_name = (char *) "pedestrian";

                break;

            case carmen_moving_object_type::car:
                moving_objects_point_clouds_message.point_clouds[i].geometric_model = 0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.height = 1.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.length = 4.5;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.width = 1.6;
                moving_objects_point_clouds_message.point_clouds[i].model_features.red = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.green = 0.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.blue = 0.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.model_name = (char *) "car";

                break;
            default:
                moving_objects_point_clouds_message.point_clouds[i].geometric_model = 0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.height = 1.8;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.length = 4.5;
                moving_objects_point_clouds_message.point_clouds[i].model_features.geometry.width = 1.6;
                moving_objects_point_clouds_message.point_clouds[i].model_features.red = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.green = 1.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.blue = 0.0;
                moving_objects_point_clouds_message.point_clouds[i].model_features.model_name = (char *) "other";


                break;
        }

        moving_objects_point_clouds_message.point_clouds[i].num_associated = clusters[i].track_id;

        // fill the points
        moving_objects_point_clouds_message.point_clouds[i].point_size = clusters[i].points.size();
        moving_objects_point_clouds_message.point_clouds[i].points = (carmen_vector_3D_t *)
                malloc(moving_objects_point_clouds_message.point_clouds[i].point_size * sizeof(carmen_vector_3D_t));
        for (int j = 0; j < moving_objects_point_clouds_message.point_clouds[i].point_size; j++) {
            //TODO modificar isso
            carmen_vector_3D_t p;

            p.x = clusters[i].points[j].x;
            p.y = clusters[i].points[j].y;
            p.z = clusters[i].points[j].z;

            offset.x = 0.572;
            offset.y = 0.0;
            offset.z = 2.154;

            p = translate_point(p, offset);

            p = rotate_point(p, globalpos.theta);

            offset.x = globalpos.x;
            offset.y = globalpos.y;
            offset.z = 0.0;

            p = translate_point(p, offset);

            moving_objects_point_clouds_message.point_clouds[i].points[j] = p;
        }

    }

}


vector<string>
objects_names_from_file(string const class_names_file)
{
    ifstream file(class_names_file);
    vector<string> file_lines;

    if (!file.is_open())
    	return file_lines;

    for (string line; getline(file, line);)
    	file_lines.push_back(line);

    cout << "object names loaded \n";

    return file_lines;
}


void
show_detections(cv::Mat *rgb_image, vector<vector<carmen_velodyne_points_in_cam_with_obstacle_t>> laser_points_in_camera_box_list,
		vector<bbox_t> predictions, double hood_removal_percentage, double fps,
                vector<carmen_position_t> rddf_points, string window_name)
{
    char confianca[25];
    char frame_rate[25];

    sprintf(frame_rate, "FPS = %.2f", fps);

    //cv::putText(*rgb_image, frame_rate, cv::Point(10, 25), cv::FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {

//		for (unsigned int j = 0; j < laser_points_in_camera_box_list[i].size(); j++)
//			cv::circle(*rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
//					laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1, cv::Scalar(0, 0, 255), 1);

        cv::Scalar object_color;

        sprintf(confianca, "%d  %.3f", predictions.at(i).obj_id, predictions.at(i).prob);

        int obj_id = predictions.at(i).obj_id;

        string obj_name;
        if (obj_names.size() > obj_id)
            obj_name = obj_names[obj_id];
//
        if (obj_name.compare("car") == 0)
        {
        	object_color = cv::Scalar(0, 0, 255);
        	cv::rectangle(*rgb_image,
        			cv::Point(predictions[i].x, predictions[i].y),
					cv::Point(predictions[i].x + predictions[i].w, predictions[i].y + predictions[i].h),
					object_color, 2);

        	cv::putText(*rgb_image, obj_name,
        			cv::Point(predictions[i].x + 1, predictions[i].y - 3),
					cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 255), 1);

        	cv::putText(*rgb_image, confianca,
        			cv::Point(predictions[i].x + 1, predictions[i].y - 3),
					cv::FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);
        }

//        if (obj_name.compare("car") == 0)
//            object_color = cv::Scalar(0, 0, 255);
//        else
//            object_color = cv::Scalar(255, 0, 255);
//
//        cv::rectangle(rgb_image,
//                      cv::Point(predictions[i].x, predictions[i].y),
//                      cv::Point(predictions[i].x + predictions[i].w, predictions[i].y + predictions[i].h),
//                      object_color, 1);
//
//        cv::putText(rgb_image, obj_name,
//                    cv::Point(predictions[i].x + 1, predictions[i].y - 3),
//                    cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 255), 1);
//
//        cv::putText(rgb_image, confianca,
//                    cv::Point(predictions[i].x + 1, predictions[i].y - 3),
//                    cv::FONT_HERSHEY_PLAIN, 1, cvScalar(255, 255, 0), 1);

    }

//    int thickness = -1;
//    int lineType = 8;
//    for (unsigned int i = 0; i < rddf_points.size(); i++)
//    {
//    	cv::circle(*rgb_image, cv::Point(rddf_points[i].x, rddf_points[i].y), 1.5, cv::Scalar(0, 255, 255), thickness, lineType);
//    }



    //cv::Mat resized_image(cv::Size(640, 480 - 480 * hood_removal_percentage), CV_8UC3);
    //cv::resize(rgb_image, resized_image, resized_image.size());
    //cv::resize(rgb_image, rgb_image, cv::Size(640, 364));
    cv::imshow(window_name, *rgb_image);
    cv::waitKey(1);

    //resized_image.release();
}


vector<cv::Scalar>
get_slice_colors (unsigned int slices_size)
{
	vector<cv::Scalar> colors;
	cv::Scalar color;
	if (slices_size== 1)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
	}

	else if (slices_size== 2)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
	}

	else if (slices_size== 3)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 0);
		colors.push_back(color);
	}

	else if (slices_size== 4)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 255, 0);
		colors.push_back(color);
	}

	else if (slices_size== 5)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 255);
		colors.push_back(color);
	}

	else if (slices_size== 6)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 255);
		colors.push_back(color);
	}

	else if (slices_size== 7)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 0, 0);
		colors.push_back(color);
	}

	else if (slices_size== 8)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 255, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 0, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 255, 255);
		colors.push_back(color);
		color = cv::Scalar (0, 0, 0);
		colors.push_back(color);
		color = cv::Scalar (255, 255, 255);
		colors.push_back(color);
	}

	return (colors);

}



bool before_first_file = true;
bool acessessing = false;
void
save_detections(double timestamp, vector<bbox_t> bounding_boxes_of_slices_in_original_image, cv::Mat rgb_image,
				vector<cv::Mat> scene_slices, vector<cv::Scalar> colors, vector<t_transform_factor> transform_factor_of_slice_to_original_frame,
				vector<carmen_position_t> rddf_points_in_image_filtered, vector<carmen_position_t> rddf_points_in_image_full)
{
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);
	char arr[50];
	char gt_path[200];
	strcpy(gt_path, groundtruth_path);
	//memcpy(arr,&timestamp,sizeof(timestamp));
	sprintf(gt_path,"%s/%lf", gt_path, timestamp);
	sprintf(arr,"%lf", timestamp);
	string str_arr (arr);
	string str_gt_path (gt_path);
	string groundtruth_folder = str_gt_path + "-r.txt";
	string detections_folder = str_folder_name + arr + "-r.txt";
	string images_folder = str_folder_image_name_slices + arr + "-r";
	string images_folder_det = str_folder_image_name_slices_and_detection + arr + "-r";
	string images_folder_original = str_folder_image_name_original + arr + "-r.png";
	string images_folder_rddf_filtered = str_folder_image_name_rddf_filtered + arr + "-r.png";
	string images_folder_rddf_full = str_folder_image_name_rddf_full + arr + "-r.png";
	string images_folder_slices_rddf_full = str_folder_image_name_slices_rddf_full + arr + "-r.png";
	string images_folder_slices_rddf_filtered = str_folder_image_name_slices_rddf_filtered + arr + "-r.png";
	cv::Mat rgb_image_original = rgb_image.clone();
	cv::Mat rgb_image_all_slices = rgb_image.clone();




//				command = "mkdir " + str_folder_image_name + "slices_and_detection/";
//				str_folder_image_name_slices_and_detection = str_folder_image_name + "slices_and_detection/";
//				retorno = system(command.c_str());

	if (access(groundtruth_folder.c_str(), F_OK) == 0)
	{
//		if (strcmp(detection_type,"-cs") == 0)
//		{
//			int thickness = -1;
//			int lineType = 8;
//
//			cv::imwrite (images_folder_original, or_image);
//
//			cv::Mat b;
//			b = or_image.clone();
//			for (unsigned int i = 0; i < rddf_points_in_image_full.size(); i++)
//			{
//				if (i % 2 == 0)
//					cv::circle(b, cv::Point(rddf_points_in_image_full[i].x, rddf_points_in_image_full[i].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
//			}
//			cv::imwrite(images_folder_rddf_full, b);
//
//
//			if(qtd_crops != 1)
//			{
//				cv::Mat c;
//				c = or_image.clone();
//				for (unsigned int i = 0; i < qtd_crops; i++)
//				{
//					cv::circle(c, cv::Point(rddf_points_in_image_filtered[i].x, rddf_points_in_image_filtered[i].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
//				}
//				cv::imwrite(images_folder_rddf_filtered, c);
//				for (int i = qtd_crops-1; i >= 1 ; i--)
//				{
//					stringstream ss;
//					ss << i;
//					string save;
//					cv::Mat open_cv_image_crop;
//					int crop_X = transform_factor_of_slice_to_original_frame[i].translate_factor_x;
//					int crop_Y = transform_factor_of_slice_to_original_frame[i].translate_factor_y;
//					int crop_w = scene_slices[i].cols;
//					int crop_h = scene_slices[i].rows;
//					//cout<<crop_X<<" "<<crop_Y<<" "<<crop_w<<" "<<crop_h<<endl;
//					cv::Mat open_cv_image = rgb_image.clone();              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
//					cv::Rect myROI(crop_X, crop_Y, crop_w, crop_h);     // TODO put this in the .ini file
//					open_cv_image_crop = open_cv_image(myROI);
//					save = images_folder_det + ss.str() + ".png";
//					cv::imwrite(save, open_cv_image_crop);
//					cv::rectangle(or_image,
//									cv::Point(transform_factor_of_slice_to_original_frame[i].translate_factor_x, transform_factor_of_slice_to_original_frame[i].translate_factor_y),
//									cv::Point(transform_factor_of_slice_to_original_frame[i].translate_factor_x + scene_slices[i].cols, transform_factor_of_slice_to_original_frame[i].translate_factor_y + scene_slices[i].rows),
//									colors[i-1], 3);
//					cv::rectangle(rgb_image_all_slices,
//							cv::Point(transform_factor_of_slice_to_original_frame[i].translate_factor_x, transform_factor_of_slice_to_original_frame[i].translate_factor_y),
//							cv::Point(transform_factor_of_slice_to_original_frame[i].translate_factor_x + scene_slices[i].cols, transform_factor_of_slice_to_original_frame[i].translate_factor_y + scene_slices[i].rows),
//							colors[i-1], 3);
//				}
//				string images_folder = str_folder_image_name_slices + arr + "-r.png";
//				images_folder_det = images_folder_det + arr + "-r.png";
//				cv::imwrite(images_folder_det, rgb_image_all_slices);
//				cv::imwrite (images_folder, or_image);
//
//				cv::Mat rgb_image_slices = rgb_image.clone();
//
//				cv::Mat a;
//				a = rgb_image_all_slices.clone();
//				for (unsigned int i = 0; i < qtd_crops; i++)
//				{
//					cv::circle(or_image, cv::Point(rddf_points_in_image_filtered[i].x, rddf_points_in_image_filtered[i].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
//				}
//				cv::imwrite(images_folder_slices_rddf_filtered, or_image);
//
//
//				for (unsigned int i = 0; i < rddf_points_in_image_full.size(); i++)
//				{
//					if (i % 2 == 0){
//						//cv::circle(rgb_image_all_slices, cv::Point(rddf_points_in_image_full[i].x, rddf_points_in_image_full[i].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
//						cv::circle(or_image, cv::Point(rddf_points_in_image_full[i].x, rddf_points_in_image_full[i].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
//					}
//				}
//				cv::imwrite(images_folder_slices_rddf_full, or_image);
//			}
//
//
//
//
////			cv::Mat a;
////			a = rgb_image_slices.clone();
////			for (unsigned int i = 0; i < rddf_points_in_image_filtered.size(); i++)
////			{
////				cv::circle(a, cv::Point(rddf_points_in_image_filtered[i].x, rddf_points_in_image_filtered[i].y), 2.5, cv::Scalar(0, 255, 255), thickness, lineType);
////			}
////			cv::imwrite(images_folder_slices_rddf_filtered, a, compression_params);
////
////			cv::Mat b;
////			b = rgb_image_original.clone();
////			for (unsigned int i = 0; i < rddf_points_in_image_full.size(); i++)
////			{
////				cv::circle(b, cv::Point(rddf_points_in_image_full[i].x, rddf_points_in_image_full[i].y), 1.5, cv::Scalar(0, 255, 255), thickness, lineType);
////			}
////			cv::imwrite(images_folder_rddf_full, b, compression_params);
//
//
//
//		}

		before_first_file = false;
				acessessing = true;
				FILE *f_groundtruth = fopen (groundtruth_folder.c_str(), "r");
				struct stat st;
				stat(groundtruth_folder.c_str(), &st);
				int size = st.st_size;
				if (size == 0)
				{
					FILE *f_detection = fopen (detections_folder.c_str(), "w");
					fclose (f_detection);
					fclose (f_groundtruth);
				}
				else
				{
					char classe [10];
					float x1, y1, x2, y2;
					int retorno = fscanf (f_groundtruth, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2);
					FILE *f_detection = fopen (detections_folder.c_str(), "w");
					for (int i = 0; i < bounding_boxes_of_slices_in_original_image.size(); i++)
					{
						//cout<<"\t"<<i<<endl;
						bbox_t b = bounding_boxes_of_slices_in_original_image[i];
						int obj_id = b.obj_id;
						//cout<<"\t"<<" "<<obj_names[obj_id]<<" "<<(float)b.x<<" "<<(float)b.y<<" "<<(float)(b.x + b.w)<<" "<<(float)(b.y + b.h)<<endl;
						string obj_name;
						if (obj_names.size() > obj_id)
							obj_name = obj_names[obj_id];

						if (obj_name.compare("car") == 0)
						{
							cv::Rect rect_A;
							cv::Rect rect_B;
							rect_A.x = (int)x1; rect_A.y = (int)y1;
							rect_A.width = (int)x2 - (int)x1; rect_A.height = (int)y2 - (int)y1;
							rect_B.x = (int)b.x; rect_B.y = (int)b.y;
							rect_B.width = (int)b.w; rect_B.height = b.h;
		//					cv::Point l1, r1, l2, r2;
		//					l1.x = (int)x1; l1.y = (int)y1; //top left
		//					r1.x = (int)x2; r1.y = (int)y2; //right botton of groundtruth bbox
		//					l2.x = (int)b.x; l2.y = (int)b.y; //top left
		//					r2.x = (int)b.x + b.w; r2.y = (int)b.y + b.h; //right botton of detection
							//cout<<x1<<" "<<x1<<" "
							//cout<<classe<<" "<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<endl;
							//if(rectangles_intersects(l1, r1, l2, r2))
							if(rectangles_intersects(rect_A, rect_B))
							{
								//cout<<"\t"<<i<<" "<<obj_names[obj_id]<<endl;
								fprintf (f_detection, "%s %f %.2f %.2f %.2f %.2f\n", "car", b.prob, (float)b.x, (float)b.y, (float)(b.x + b.w), (float)(b.y + b.h));
							}
						}

					}
					fclose (f_detection);
					fclose (f_groundtruth);
		}
	}
	else
		acessessing = false;

	if (before_first_file == false && acessessing == false)
	{
		cout<<"database_completed!"<<endl;
		exit(0);
	}
}


//bool before_first_file = true;
//bool acessessing = false;
//void
//save_detections(double timestamp, vector<bbox_t> bounding_boxes_of_slices_in_original_image, cv::Mat rgb_image,
//				vector<cv::Mat> scene_slices, vector<cv::Scalar> colors, vector<t_transform_factor> transform_factor_of_slice_to_original_frame,
//				vector<carmen_position_t> rddf_points_in_image_filtered, vector<carmen_position_t> rddf_points_in_image_full)
//{
//	char arr[50];
//	char gt_path[200];
//	strcpy(gt_path, groundtruth_path);
//	//memcpy(arr,&timestamp,sizeof(timestamp));
//	sprintf(gt_path,"%s/%lf", gt_path, timestamp);
//	sprintf(arr,"%lf", timestamp);
//	string str_arr (arr);
//	string str_gt_path (gt_path);
//	string groundtruth_folder = str_gt_path + "-r.txt";
//	string detections_folder = str_folder_name + arr + "-r.txt";
//	string images_folder = str_folder_image_name_slices + arr + "-r.png";
//	string images_folder_rddf_filtered = str_folder_image_name_rddf_filtered + arr + "-r.png";
//	string images_folder_rddf_full = str_folder_image_name_rddf_full + arr + "-r.png";
//	string images_folder_slices_rddf_full = str_folder_image_name_slices_rddf_full + arr + "-r.png";
//	string images_folder_slices_rddf_filtered = str_folder_image_name_slices_rddf_filtered + arr + "-r.png";
//	cv::Mat rgb_image_original = rgb_image.clone();
//
//	string or_image_folder = str_arr + "-r(ORIGINAL).png";
//	string or_image_w_crops = str_arr + "-r(ORIGINAL_WITH_CROPS).png";
//	string or_image_w_crops_waypoint = str_arr + "-r(ORIGINAL_WITH_CROPS_WAYPOINT).png";
//	cv::Mat or_image_cropped = or_image.clone();
//
//	//if (access(groundtruth_folder.c_str(), F_OK) == 0)
//	//{
//		if (strcmp(detection_type,"-cs") == 0)
//		{
//
//			imwrite(or_image_folder, or_image);
//			for (int i = qtd_crops-1; i >= 1 ; i--)
//			{
//				cv::rectangle(rgb_image,
//						cv::Point(transform_factor_of_slice_to_original_frame[i].translate_factor_x, transform_factor_of_slice_to_original_frame[i].translate_factor_y),
//						cv::Point(transform_factor_of_slice_to_original_frame[i].translate_factor_x + scene_slices[i].cols, transform_factor_of_slice_to_original_frame[i].translate_factor_y + scene_slices[i].rows),
//						colors[i-1], 3);
//			}
//			cv::imwrite(images_folder, rgb_image);
//
//			for (int i = qtd_crops-1; i >= 1 ; i--)
//			{
//				cv::rectangle(or_image_cropped,
//						cv::Point(transform_factor_of_slice_to_original_frame[i].translate_factor_x, transform_factor_of_slice_to_original_frame[i].translate_factor_y),
//						cv::Point(transform_factor_of_slice_to_original_frame[i].translate_factor_x + scene_slices[i].cols, transform_factor_of_slice_to_original_frame[i].translate_factor_y + scene_slices[i].rows),
//						colors[i-1], 3);
//			}
//			cv::imwrite(or_image_w_crops, or_image_cropped);
//
//			cv::Mat rgb_image_slices = rgb_image.clone();
//
//			int thickness = -1;
//			int lineType = 8;
//			for (unsigned int i = 0; i < rddf_points_in_image_full.size(); i++)
//			{
//				cv::circle(rgb_image, cv::Point(rddf_points_in_image_full[i].x, rddf_points_in_image_full[i].y), 1.5, cv::Scalar(0, 255, 255), thickness, lineType);
//			}
//			cv::imwrite(images_folder_slices_rddf_full, rgb_image);
//
//			cv::Mat a;
//			a = rgb_image_slices.clone();
//			for (unsigned int i = 0; i < rddf_points_in_image_filtered.size(); i++)
//			{
//				cv::circle(a, cv::Point(rddf_points_in_image_filtered[i].x, rddf_points_in_image_filtered[i].y), 2.5, cv::Scalar(0, 255, 255), thickness, lineType);
//			}
//			cv::imwrite(images_folder_slices_rddf_filtered, a);
//
//			cv::Mat b;
//			//b = rgb_image_original.clone();
//			b = or_image.clone();
//			for (unsigned int i = 0; i < rddf_points_in_image_full.size(); i++)
//			{
//				if(i%2 == 0)
//					cv::circle(b, cv::Point(rddf_points_in_image_full[i].x, rddf_points_in_image_full[i].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
//			}
//			cv::imwrite(images_folder_rddf_full, b);
//
//			cv::Mat d;
//			//b = rgb_image_original.clone();
//			d = or_image_cropped.clone();
//			for (unsigned int i = 0; i < rddf_points_in_image_full.size(); i++)
//			{
//				if(i%2 == 0)
//					cv::circle(d, cv::Point(rddf_points_in_image_full[i].x, rddf_points_in_image_full[i].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
//			}
//			cv::imwrite(or_image_w_crops_waypoint, d);
//
//			cv::Mat c;
//			//c = rgb_image_original.clone();
//			c = or_image.clone();
//			for (unsigned int i = 0; i < rddf_points_in_image_filtered.size(); i++)
//			{
//				cv::circle(c, cv::Point(rddf_points_in_image_filtered[i].x, rddf_points_in_image_filtered[i].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
//			}
//			cv::imwrite(images_folder_rddf_filtered, c);
//
//		}
//
////		before_first_file = false;
////		acessessing = true;
////		FILE *f_groundtruth = fopen (groundtruth_folder.c_str(), "r");
////		struct stat st;
////		stat(groundtruth_folder.c_str(), &st);
////		int size = st.st_size;
////		if (size == 0)
////		{
////			FILE *f_detection = fopen (detections_folder.c_str(), "w");
////			fclose (f_detection);
////			fclose (f_groundtruth);
////		}
////		else
////		{
////			char classe [10];
////			float x1, y1, x2, y2;
////			int retorno = fscanf (f_groundtruth, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2);
////			FILE *f_detection = fopen (detections_folder.c_str(), "w");
////			for (int i = 0; i < bounding_boxes_of_slices_in_original_image.size(); i++)
////			{
////				//cout<<"\t"<<i<<endl;
////				bbox_t b = bounding_boxes_of_slices_in_original_image[i];
////				int obj_id = b.obj_id;
////				//cout<<"\t"<<" "<<obj_names[obj_id]<<" "<<(float)b.x<<" "<<(float)b.y<<" "<<(float)(b.x + b.w)<<" "<<(float)(b.y + b.h)<<endl;
////				string obj_name;
////				if (obj_names.size() > obj_id)
////					obj_name = obj_names[obj_id];
////
////				if (obj_name.compare("car") == 0)
////				{
////					cv::Rect rect_A;
////					cv::Rect rect_B;
////					rect_A.x = (int)x1; rect_A.y = (int)y1;
////					rect_A.width = (int)x2 - (int)x1; rect_A.height = (int)y2 - (int)y1;
////					rect_B.x = (int)b.x; rect_B.y = (int)b.y;
////					rect_B.width = (int)b.w; rect_B.height = b.h;
//////					cv::Point l1, r1, l2, r2;
//////					l1.x = (int)x1; l1.y = (int)y1; //top left
//////					r1.x = (int)x2; r1.y = (int)y2; //right botton of groundtruth bbox
//////					l2.x = (int)b.x; l2.y = (int)b.y; //top left
//////					r2.x = (int)b.x + b.w; r2.y = (int)b.y + b.h; //right botton of detection
////					//cout<<x1<<" "<<x1<<" "
////					//cout<<classe<<" "<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<endl;
////					//if(rectangles_intersects(l1, r1, l2, r2))
////					if(rectangles_intersects(rect_A, rect_B))
////					{
////						//cout<<"\t"<<i<<" "<<obj_names[obj_id]<<endl;
////						fprintf (f_detection, "%s %f %.2f %.2f %.2f %.2f\n", "car", b.prob, (float)b.x, (float)b.y, (float)(b.x + b.w), (float)(b.y + b.h));
////					}
////				}
////
////			}
////			fclose (f_detection);
////			fclose (f_groundtruth);
////		}
////	//}
////	//else
////	//	acessessing = false;
////
//////	if (before_first_file == false && acessessing == false)
//////	{
//////		cout<<"database_completed!"<<endl;
//////		exit(0);
//////	}
//}


void
detections(vector<bbox_t> predictions, carmen_bumblebee_basic_stereoimage_message *image_msg, carmen_velodyne_partial_scan_message velodyne_sync_with_cam,
		   cv::Mat src_image, cv::Mat *rgb_image, double start_time, double fps, vector<carmen_position_t> rddf_points,
		   string window_name)
{
	vector <bounding_box> bouding_boxes_list;
	double hood_removal_percentage = 0.2;
	vector<carmen_tracked_cluster_t> clusters;

	//predictions = darknet->tracking(predictions); // Coment this line if object tracking is not necessary

	vector<vector<carmen_velodyne_points_in_cam_with_obstacle_t>> laser_points_in_camera_box_list;

//	for (const auto &box : predictions) // Covert Darknet bounding box to neural_object_deddtector bounding box
//	{
//		bounding_box bbox;
//
//		bbox.pt1.x = box.x;
//		bbox.pt1.y = box.y;
//		bbox.pt2.x = box.x + box.w;
//		bbox.pt2.y = box.y + box.h;
//
//		bouding_boxes_list.push_back(bbox);
//	}
//
//
//	// Removes the ground, Removes points outside cameras field of view and Returns the points that are obstacles and are inside bboxes
//	laser_points_in_camera_box_list = velodyne_points_in_boxes(bouding_boxes_list,
//			&velodyne_sync_with_cam, camera_parameters, velodyne_pose, camera_pose, image_msg->width, image_msg->height);
//
//
//	// ONLY Convert from sferical to cartesian cordinates
//	vector< vector<carmen_vector_3D_t>> cluster_list = get_cluster_list(laser_points_in_camera_box_list);
//
//	// Cluster points and get biggest
//	filter_points_in_clusters(&cluster_list);
//
//	for (int i = 0; i < cluster_list.size(); i++)
//	{
//		carmen_moving_object_type tp = find_cluster_type_by_obj_id(obj_names, predictions.at(i).obj_id);
//
//		int cluster_id = predictions.at(i).track_id;
//
//		carmen_tracked_cluster_t clust;
//
//		clust.points = cluster_list.at(i);
//
//		clust.orientation = globalpos.theta;  //TODO: Calcular velocidade e orientacao corretas (provavelmente usando um tracker)
//		clust.linear_velocity = 0.0;
//		clust.track_id = cluster_id;
//		clust.last_detection_timestamp = image_msg->timestamp;
//		clust.cluster_type = tp;
//
//		clusters.push_back(clust);
//	}
//
//	build_moving_objects_message(clusters);

	fps = 1.0 / (carmen_get_time() - start_time);
	start_time = carmen_get_time();

#ifdef SHOW_DETECTIONS
	show_detections(rgb_image, laser_points_in_camera_box_list, predictions,
			hood_removal_percentage, fps, rddf_points, window_name);
#endif
}


bool
check_rect_inside_image (cv::Rect rec, cv::Mat img)
{
	if(0 <= rec.x
		&& 0 <= rec.width
		&& rec.x + rec.width <= img.cols
		&& 0 <= rec.y
		&& 0 <= rec.height
		&& rec.y + rec.height <= img.rows)
	{
		return true;
	}

	return false;
}


int
calc_area (int width, int height)
{
	return (width * height);
}


float
calc_percentage_of_rectangles_intersection(cv::Point l1, cv::Point r1, cv::Point l2, cv::Point r2)
{
	float intersection_percentage;

	// Area of 1st Rectangle
	int area1 = abs( calc_area((l1.x - r1.x), (l1.y - r1.y)) ); //abs(l1.x - r1.x) * abs(l1.y - r1.y);
	// Area of 2nd Rectangle
	int area2 = abs( calc_area((l2.x - r2.x), (l2.y - r2.y)) ); //abs(l2.x - r2.x) * abs(l2.y - r2.y);
	//cout<<"\t"<<"Area of bBox2: "<<area2;
	int total_area;

	// Length of intersecting part i.e
	// start from max(l1.x, l2.x) of
	// x-coordinate and end at min(r1.x,
	// r2.x) x-coordinate by subtracting
	// start from end we get required
	// lengths
	int areaI = abs((min(r1.x, r2.x) - max(l1.x, l2.x)) * (min(r1.y, r2.y) - max(l1.y, l2.y)));
	total_area  = abs(area1 + area2 - areaI);


	intersection_percentage = (100 * areaI) / (total_area);
	//cout<<"\t"<<"Intersection percent: "<<intersection_percentage<<endl;
	return (intersection_percentage);


}


bool rectangles_intersects2(cv::Point l1, cv::Point r1, cv::Point l2, cv::Point r2)
{


	if ((l1.x < r2.x) &&
		(r1.x > l2.x) &&
		(l1.y < r2.y) &&
		(r1.y > l2.y))
		return true;


    return false;
}


bool valueInRange(int value, int min, int max)
{ return (value >= min) && (value <= max); }

bool rectangles_intersects(cv::Rect A, cv::Rect B)
{
    bool xOverlap = valueInRange(A.x, B.x, B.x + B.width) ||
                    valueInRange(B.x, A.x, A.x + A.width);

    bool yOverlap = valueInRange(A.y, B.y, B.y + B.height) ||
                    valueInRange(B.y, A.y, A.y + A.height);

    return xOverlap && yOverlap;
}


vector<bbox_t>
transform_bounding_boxes_of_slices2 (vector<vector<bbox_t>> bounding_boxes_of_slices, vector<t_transform_factor> transform_factor_of_slice_to_original_frame)
{
	vector<bbox_t> bboxes;
	bbox_t b;
	bool intersects_with_bboxes = false;
	bool rect_dont_intersects = false;
	for (int i = 0; i < bounding_boxes_of_slices.size(); i++)
	{
		for (int j = 0; j < bounding_boxes_of_slices[i].size(); j++)

		{
			b = bounding_boxes_of_slices[i][j];
			b.x = bounding_boxes_of_slices[i][j].x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
			b.y = bounding_boxes_of_slices[i][j].y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;
			//b.w = bouding_boxes_of_slices[i][j].w / transform_factor_of_slice_to_original_frame[i].scale_factor_x;
			//b.h = bouding_boxes_of_slices[i][j].h / transform_factor_of_slice_to_original_frame[i].scale_factor_y;
			if (i == 0)
			{
				bboxes.push_back(b);
			}
			else
			{
				cv::Rect rect_B;
				rect_B.x = (int)b.x; rect_B.y = (int)b.y;
				rect_B.width = (int)b.w; rect_B.height = b.h;
				cv::Point l1; //top left
				l1.x = b.x;
				l1.y = b.y;
				cv::Point r1; //bottom right
				r1.x = b.x + b.w;
				r1.y = b.y + b.h;
				//cout<<l1.x<<" "<<l1.y<<" "<<r1.x<<" "<<r1.y<<endl;
				int area1 = abs( calc_area((l1.x - r1.x), (l1.y - r1.y)) ); //abs(l1.x - r1.x) * abs(l1.y - r1.y);
				//cout<<"Area of bBox1: "<<area1<<endl;

				for (int k = 0; k < bboxes.size(); k++)
				{
					cv::Rect rect_A;
					rect_A.x = (int)bboxes[k].x; rect_A.y = (int)bboxes[k].y;
					rect_A.width = (int)bboxes[k].w; rect_A.height = (int)bboxes[k].h;

					//					cv::Point l1, r1, l2, r2;
					//					l1.x = (int)x1; l1.y = (int)y1; //top left
					//					r1.x = (int)x2; r1.y = (int)y2; //right botton of groundtruth bbox
					//					l2.x = (int)b.x; l2.y = (int)b.y; //top left
					//					r2.x = (int)b.x + b.w; r2.y = (int)b.y + b.h; //right botton of detection
					float percentage_of_intersection_between_bboxes;
					cv::Point l2;
					l2.x = bboxes[k].x;
					l2.y = bboxes[k].y;
					cv::Point r2;
					r2.x = bboxes[k].x + bboxes[k].w;
					r2.y = bboxes[k].y + bboxes[k].h;
					//cout<<"\t"<<l2.x<<" "<<l2.y<<" "<<r2.x<<" "<<r2.y<<endl;
					//if (rectangles_intersects(l1, r1, l2, r2))
					if(rectangles_intersects(rect_A, rect_B))
					{
						percentage_of_intersection_between_bboxes = calc_percentage_of_rectangles_intersection(l1, r1, l2, r2);
						//cout<< percentage_of_intersection_between_bboxes<< endl;
						int area2 = abs( calc_area((l2.x - r2.x), (l2.y - r2.y)) ); //abs(l2.x - r2.x) * abs(l2.y - r2.y);
//						if (area2 > area1)
//						{
//							b.x = l2.x;
//							b.y = l2.y;
//							b.w = bboxes[k].w;
//							b.h = bboxes[k].h;
//						}

						if (percentage_of_intersection_between_bboxes > 1)
						{
							if(b.prob > bboxes[k].prob)
							{
								bboxes[k] = b;
								break;
							}
							intersects_with_bboxes = true;
							break;
						}
					}

				}
				if (intersects_with_bboxes == false)
				{
					bboxes.push_back(b);
					intersects_with_bboxes = false;
				}

			}

		}
	}

	return (bboxes);
}


vector<bbox_t>
transform_bounding_boxes_of_slices3 (vector<vector<bbox_t>> bounding_boxes_of_slices, vector<t_transform_factor> transform_factor_of_slice_to_original_frame)
{
	cv::Mat img;
	img = or_image.clone();
	vector<bbox_t> bboxes;
	bbox_t b;
	bool intersects_with_bboxes = false;
	bool rect_dont_intersects = false;
	int nao_intersecta = 0;
	for (int i = 0; i < bounding_boxes_of_slices.size(); i++)
	{
		for (int j = 0; j < bounding_boxes_of_slices[i].size(); j++)

		{
			b = bounding_boxes_of_slices[i][j];
			b.x = bounding_boxes_of_slices[i][j].x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
			b.y = bounding_boxes_of_slices[i][j].y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;
			int obj_id = b.obj_id;
			string obj_name;
			if (obj_names.size() > obj_id)
				obj_name = obj_names[obj_id];
			if (i == 0)
			{
				if (obj_name.compare("car") == 0)
					bboxes.push_back(b);

				cout<<"bbox 0 size: "<<bboxes.size()<<endl;
			}
			else
			{
				if (obj_name.compare("car") == 0)
				{
					cv::Rect rect_B;
					rect_B.x = (int)b.x; rect_B.y = (int)b.y;
					rect_B.width = (int)b.w; rect_B.height = b.h;
					cv::Point l1; //top left
					l1.x = b.x;	l1.y = b.y;
					cv::Point r1; //bottom right
					r1.x = b.x + b.w; r1.y = b.y + b.h;
					//cout<<l1.x<<" "<<l1.y<<" "<<r1.x<<" "<<r1.y<<endl;
					int area1 = abs( calc_area((l1.x - r1.x), (l1.y - r1.y)) ); //abs(l1.x - r1.x) * abs(l1.y - r1.y);
					//cout<<"Area of bBox1: "<<area1<<endl;

					for (int k = 0; k < bboxes.size(); k++)
					{
						cv::rectangle(img,
								cv::Point(bboxes[k].x, bboxes[k].y),
								cv::Point(bboxes[k].x + bboxes[k].w, bboxes[k].y + bboxes[k].h),
								cvScalar(255,0,0), 2);
						imshow("debug",img);
						//cv::waitKey(100);
						cv::Rect rect_A;
						rect_A.x = (int)bboxes[k].x; rect_A.y = (int)bboxes[k].y;
						rect_A.width = (int)bboxes[k].w; rect_A.height = (int)bboxes[k].h;


						float percentage_of_intersection_between_bboxes;
						cv::Point l2;
						l2.x = bboxes[k].x;	l2.y = bboxes[k].y;
						cv::Point r2;
						r2.x = bboxes[k].x + bboxes[k].w; r2.y = bboxes[k].y + bboxes[k].h;
						//cout<<"\t"<<l2.x<<" "<<l2.y<<" "<<r2.x<<" "<<r2.y<<endl;
						//if (rectangles_intersects2(l1, r1, l2, r2))
						if(rectangles_intersects(rect_A, rect_B))
						{
							cout<<endl<<"RETANGULOS INTERSECTAM!!!!!"<<endl;
							percentage_of_intersection_between_bboxes = calc_percentage_of_rectangles_intersection(l1, r1, l2, r2);
							//cout<<percentage_of_intersection_between_bboxes<<endl;

							if ( (((rect_A) & (rect_B)) != rect_A) && (((rect_B) & (rect_A)) != rect_B) )
							{
								cout<<"UM NAO ESTA DENTRO DO OUTRO!!!!!"<<endl;
								if (percentage_of_intersection_between_bboxes > 50)
								{
									cout<<"PORCENTAGEM EH MAIOR QUE 50"<<endl;
									cv::rectangle(img,
											cv::Point(b.x, b.y),
											cv::Point(b.x + b.w, b.y + b.h),
											cvScalar(255,0,255), 2);
									imshow("debug",img);
									cv::waitKey(1);
									if(b.prob > bboxes[k].prob)
									{
										cout<<"TROQUEI BBOXES!!"<<endl;
										bboxes[k] = b;
										cv::rectangle(img,
												cv::Point(bboxes[k].x, bboxes[k].y),
												cv::Point(bboxes[k].x + bboxes[k].w, bboxes[k].y + bboxes[k].h),
												cvScalar(255,255,255), 2);
										imshow("debug",img);
										break;

									}
								}

								else
								{
									nao_intersecta ++;

								}
							}
							else
								cout<<"UM ESTA DENTRO DO OUTRO!!!!!"<<endl;

						}
						else
						{
							cout<<"RETANGULOS NAO INTERSECTAM!!!!!"<<endl;
							nao_intersecta ++;
						}

					}
					if (nao_intersecta == bboxes.size())
					{
						bboxes.push_back(b);
					}

					nao_intersecta = 0;
	//				if (intersects_with_bboxes == false)
	//				{
	//					bboxes.push_back(b);
	//					intersects_with_bboxes = false;
	//				}

				}
			}
		}
	}

	cout<<"******************************"<<endl<<endl;
	return (bboxes);
}



vector<bbox_t>
transform_bounding_boxes_of_slices (vector<vector<bbox_t>> bounding_boxes_of_slices, vector<t_transform_factor> transform_factor_of_slice_to_original_frame)
{
	cv::Mat img;
	img = or_image.clone();
	vector<bbox_t> bboxes;
	bbox_t b;
	bool intersects_with_bboxes = false;
	bool rect_dont_intersects = false;
	int nao_intersecta = 0;
	for (int i = 0; i < bounding_boxes_of_slices.size(); i++)
	{
		for (int j = 0; j < bounding_boxes_of_slices[i].size(); j++)

		{
			b = bounding_boxes_of_slices[i][j];
			b.x = bounding_boxes_of_slices[i][j].x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
			b.y = bounding_boxes_of_slices[i][j].y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;
			int obj_id = b.obj_id;
			string obj_name;
			if (obj_names.size() > obj_id)
				obj_name = obj_names[obj_id];
			if (i == 0)
			{
				if (obj_name.compare("car") == 0)
					bboxes.push_back(b);
			}
			else
			{
				if (obj_name.compare("car") == 0)
				{
					cv::Rect rect_B;
					rect_B.x = (int)b.x; rect_B.y = (int)b.y;
					rect_B.width = (int)b.w; rect_B.height = b.h;
					cv::Point l1; //top left
					l1.x = b.x;
					l1.y = b.y;
					cv::Point r1; //bottom right
					r1.x = b.x + b.w;
					r1.y = b.y + b.h;
					//cout<<l1.x<<" "<<l1.y<<" "<<r1.x<<" "<<r1.y<<endl;
					int area1 = abs( calc_area((l1.x - r1.x), (l1.y - r1.y)) ); //abs(l1.x - r1.x) * abs(l1.y - r1.y);
					//cout<<"Area of bBox1: "<<area1<<endl;

					for (int k = 0; k < bboxes.size(); k++)
					{
//						cv::rectangle(img,
//								cv::Point(bboxes[k].x, bboxes[k].y),
//								cv::Point(bboxes[k].x + bboxes[k].w, bboxes[k].y + bboxes[k].h),
//								cvScalar(255,0,0), 2);
						//imshow("debug",img);
						//cv::waitKey(100);
						cv::Rect rect_A;
						rect_A.x = (int)bboxes[k].x; rect_A.y = (int)bboxes[k].y;
						rect_A.width = (int)bboxes[k].w; rect_A.height = (int)bboxes[k].h;

						//					cv::Point l1, r1, l2, r2;
						//					l1.x = (int)x1; l1.y = (int)y1; //top left
						//					r1.x = (int)x2; r1.y = (int)y2; //right botton of groundtruth bbox
						//					l2.x = (int)b.x; l2.y = (int)b.y; //top left
						//					r2.x = (int)b.x + b.w; r2.y = (int)b.y + b.h; //right botton of detection
						float percentage_of_intersection_between_bboxes;
						cv::Point l2;
						l2.x = bboxes[k].x;
						l2.y = bboxes[k].y;
						cv::Point r2;
						r2.x = bboxes[k].x + bboxes[k].w;
						r2.y = bboxes[k].y + bboxes[k].h;
						//cout<<"\t"<<l2.x<<" "<<l2.y<<" "<<r2.x<<" "<<r2.y<<endl;
						//if (rectangles_intersects2(l1, r1, l2, r2))
						if(rectangles_intersects(rect_A, rect_B))
						{
							percentage_of_intersection_between_bboxes = calc_percentage_of_rectangles_intersection(l1, r1, l2, r2);
							//cout<<percentage_of_intersection_between_bboxes<<endl;
							if ( (((rect_A) & (rect_B)) != rect_A) || (((rect_B) & (rect_A)) != rect_B) )
							{
//								cv::rectangle(img,
//										cv::Point(bboxes[k].x, bboxes[k].y),
//										cv::Point(bboxes[k].x + bboxes[k].w, bboxes[k].y + bboxes[k].h),
//										cvScalar(255,255,0), 2);
//								imshow("debug",img);
//								cv::putText(img, "UM SOBRE O OUTRO!!",
//										cv::Point(rect_A.x + 1, rect_A.y - 3),
//										cv::FONT_HERSHEY_PLAIN, 1, cvScalar(0, 0, 255), 1);
								continue;
							}


							if (percentage_of_intersection_between_bboxes > 50)
							{
//								cv::rectangle(img,
//										cv::Point(b.x, b.y),
//										cv::Point(b.x + b.w, b.y + b.h),
//										cvScalar(255,0,255), 2);
//								imshow("debug",img);
													cv::waitKey(1);
								if(b.prob >= bboxes[k].prob)
								{
									bboxes[k] = b;
//									cv::rectangle(img,
//											cv::Point(bboxes[k].x, bboxes[k].y),
//											cv::Point(bboxes[k].x + bboxes[k].w, bboxes[k].y + bboxes[k].h),
//											cvScalar(255,255,255), 2);
//									imshow("debug",img);
									break;

								}
							}
							else
							{
								nao_intersecta ++;
							}
						}
						else
						{
							nao_intersecta ++;
						}

					}
					if (nao_intersecta == bboxes.size())
					{
						bboxes.push_back(b);
					}

					nao_intersecta = 0;
	//				if (intersects_with_bboxes == false)
	//				{
	//					bboxes.push_back(b);
	//					intersects_with_bboxes = false;
	//				}

				}
			}
		}
	}

	return (bboxes);
}


vector<bbox_t>
get_predictions_of_slices(int i, cv::Mat image)
{
	vector<bbox_t> predictions;
	predictions = darknet->detect(image, 0.2);  // Arguments (img, threshold)
	return (predictions);
}


void
get_image_slices(vector<cv::Mat> &scene_slices, vector<t_transform_factor> &transform_factor_of_slice_to_original_frame,
		cv::Mat out, vector<carmen_position_t> rddf_points_in_image,
		vector<double> distances_of_rddf_from_car)
{
	cv::Mat roi;
	cv::Point top_left_point;
	t_transform_factor t;
	int sum_transform_x = 0;
	int sum_transform_y = 0;
	double mult_scale_x = 0;
	double mult_scale_y = 0;

	float first_crop_percentage = 0.6;
	int image_size_x = scene_slices[0].cols * first_crop_percentage;
	int image_size_y = scene_slices[0].rows * first_crop_percentage;
	double scale = image_size_y*0.75;
	top_left_point.x = static_cast<double>(rddf_points_in_image[0].x) - (image_size_x/2);
	top_left_point.y = static_cast<double>(rddf_points_in_image[0].y)-scale;
//	cout<<top_left_point.x<<" "<<top_left_point.y<<endl;
	cv::Rect rec(top_left_point.x, top_left_point.y, image_size_x, image_size_y);
	if (check_rect_inside_image(rec, out))
	{
		roi = out (rec);
		mult_scale_x += double(scene_slices[0].cols) / double(roi.cols);
		mult_scale_y += double(scene_slices[0].rows) / double(roi.rows);
		t.scale_factor_x = mult_scale_x;
		t.scale_factor_y = mult_scale_y;
		sum_transform_x += top_left_point.x;
		sum_transform_y += top_left_point.y;
		t.translate_factor_x = sum_transform_x;
		t.translate_factor_y = sum_transform_y;
//		cout<<t.translate_factor_x<<" "<<t.translate_factor_y<<endl;
		scene_slices.push_back(roi);
		transform_factor_of_slice_to_original_frame.push_back(t);
	}

	//cout<<rddf_points_in_image.size()<<" "<<distances_of_rddf_from_car.size()<<endl;
	for (int i = 1; i < rddf_points_in_image.size(); i++)
	{
		image_size_x = (scene_slices[0].cols * first_crop_percentage) / (i+1);
		image_size_y = (scene_slices[0].rows * first_crop_percentage) / (i+1);
		if (image_size_x > 1)//
		{
			double scale = image_size_y*(0.75);
			top_left_point.x = static_cast<double>(rddf_points_in_image[i].x) - (image_size_x/2);
			top_left_point.y = static_cast<double>(rddf_points_in_image[i].y) - scale;
			cv::Rect rec(top_left_point.x, top_left_point.y, image_size_x, image_size_y);
			if (check_rect_inside_image(rec, out))
			{
				roi = out (rec);
				mult_scale_x += double(scene_slices[0].cols) / double(roi.cols);
				mult_scale_y += double(scene_slices[0].rows) / double(roi.rows);
				t.scale_factor_x = mult_scale_x;
				t.scale_factor_y = mult_scale_y;
				sum_transform_x = top_left_point.x;
				sum_transform_y = top_left_point.y;
				t.translate_factor_x = sum_transform_x;
				t.translate_factor_y = sum_transform_y;
//				cout<<t.translate_factor_x<<" "<<t.translate_factor_y<<endl;
				scene_slices.push_back(roi);
				transform_factor_of_slice_to_original_frame.push_back(t);
			}

		}

	}
}


double
euclidean_distance (double x1, double x2, double y1, double y2)
{
	return ( sqrt(pow(x2-x1,2) + pow(y2-y1,2)) );
}


vector<carmen_position_t>
get_rddf_points_in_image_full (tf::StampedTransform world_to_camera_pose, int img_width, int img_height)
{
	carmen_position_t p;
	vector<carmen_position_t> rddf_points_in_image;
	double distance, last_distance;
	for(int i = 0; i < last_rddf_poses.number_of_poses; i++)
	{
		p = convert_rddf_pose_to_point_in_image (last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, 0.0, world_to_camera_pose, camera_parameters, img_width, img_height);
		rddf_points_in_image.push_back(p);
	}
	return (rddf_points_in_image);
}


void
carmen_translate_2d(double *x, double *y, double offset_x, double offset_y)
{
	*x += offset_x;
	*y += offset_y;
}


int
get_closest_rddf_index(double *camera_pose_x, double *camera_pose_y)
{
	carmen_point_t globalpos_in_camera;
	*camera_pose_x = camera_pose.position.x;
	*camera_pose_y = camera_pose.position.y;

	carmen_translate_2d(camera_pose_x, camera_pose_y, board_pose.position.x, board_pose.position.y);
	carmen_rotate_2d  (camera_pose_x, camera_pose_y, carmen_normalize_theta(globalpos.theta));
	carmen_translate_2d(camera_pose_x, camera_pose_y, globalpos.x, globalpos.y);


	int index_aux;
	double distance, min_distance = DBL_MAX;
	for (int i = 0; i < last_rddf_poses.number_of_poses; i++)
	{
		distance = euclidean_distance(*camera_pose_x, last_rddf_poses.poses[i].x, *camera_pose_y, last_rddf_poses.poses[i].y);
		if (distance < min_distance)
		{
			min_distance = distance;
			index_aux = i;
		}
	}
	return index_aux;

}


vector<carmen_position_t>
get_rddf_points_in_image(double meters_spacement, vector<double> &distances_of_rddf_from_car, tf::StampedTransform world_to_camera_pose, int img_width, int img_height)
{

	carmen_position_t p;
	vector<carmen_position_t> rddf_points_in_image;

	int inicial_rddf_index;
	double camera_pose_x, camera_pose_y;
	inicial_rddf_index = get_closest_rddf_index(&camera_pose_x, &camera_pose_y);

	double distance = 0.0, distance_ant = 0.0;
	double distance_accum = meters_spacement;

	for (int i = inicial_rddf_index; i < last_rddf_poses.number_of_poses; i++)
	{
		distance = euclidean_distance(camera_pose_x, last_rddf_poses.poses[i].x, camera_pose_y, last_rddf_poses.poses[i].y);
//		printf("Distance of cam_point %d to camera: %lf\n", i,distance);
		if (distance > distance_accum)
		{
			int closest_point;
			if (abs(distance_accum - distance) < abs(distance_accum - distance_ant))
				closest_point = i;
			else
				closest_point = i - 1;

			distance_accum += meters_spacement;

			p = convert_rddf_pose_to_point_in_image (last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, 0.0, world_to_camera_pose, camera_parameters, img_width, img_height);
			distances_of_rddf_from_car.push_back(distance);
			rddf_points_in_image.push_back(p);
			distance_ant = distance;
		}
	}

	return (rddf_points_in_image);
}


carmen_pose_3D_t
filter_pitch(carmen_pose_3D_t car_pose)
{
	carmen_pose_3D_t filtered_car_pose;
	//cout<<car_pose.orientation.pitch<<" ";
	filtered_car_pose = car_pose;
	SampleFilter_put(&filter2, pose.orientation.pitch);
	filtered_car_pose.orientation.pitch = SampleFilter_get(&filter2);
	//cout<<filtered_car_pose.orientation.pitch<<endl;
	filtered_car_pose.orientation.pitch = 0.0;

	return (filtered_car_pose);
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_moving_objects_message(double timestamp)
{
    moving_objects_point_clouds_message.timestamp = timestamp;
    moving_objects_point_clouds_message.host = carmen_get_host();

    carmen_moving_objects_point_clouds_publish_message(&moving_objects_point_clouds_message);

    for (int i = 0; i < moving_objects_point_clouds_message.num_point_clouds; i++) {
        free(moving_objects_point_clouds_message.point_clouds[i].points);
    }
    free(moving_objects_point_clouds_message.point_clouds);
}
///////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
rddf_handler(carmen_behavior_selector_road_profile_message *message)
{
	last_rddf_poses = *message;
}


static void
rddf_annotation_message_handler(carmen_rddf_annotation_message *message)
{
	last_rddf_annotation_message = *message;
	last_rddf_annotation_message_valid = true;
}


#define crop_x 0.0
#define crop_y 1.0


void
image_handler2(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	carmen_velodyne_partial_scan_message velodyne_sync_with_cam;
	carmen_laser_ldmrs_new_message sick_sync_with_cam;

	if (image_msg == NULL)
		return;

	if (velodyne_vector.size() > 0)
		velodyne_sync_with_cam = find_velodyne_most_sync_with_cam(image_msg->timestamp);
	else
		return;

	if (sick_vector.size() > 0)
		sick_sync_with_cam = find_sick_most_sync_with_cam(image_msg->timestamp);
	else
		return;


	double fps;
	static double start_time = 0.0;
	unsigned char *img;

	if (camera_side == 0)
		img = image_msg->raw_left;
	else
		img = image_msg->raw_right;

	int crop_X = 0;
	int crop_Y = 0;
	int crop_w = image_msg->width;// 1280;
	int crop_h = image_msg->height*0.8;//400; // 500;


	cv::Mat open_cv_image = cv::Mat(image_msg->height, image_msg->width, CV_8UC3, img, 0);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
	cv::Rect myROI(crop_X, crop_Y, crop_w, crop_h);     // TODO put this in the .ini file
	open_cv_image = open_cv_image(myROI);

//	vector<carmen_velodyne_points_in_cam_t> sick_points = carmen_sick_camera_calibration_lasers_points_in_camera(sick_laser_message,
//														  camera_parameters,
//			                                              &transformer_sick,
//														  open_cv_image.cols, image_msg->height);

	vector<image_cartesian> sick_points = sick_camera_calibration_fuse_camera_lidar(&sick_sync_with_cam, camera_parameters, &transformer_sick,
			image_msg->width, image_msg->height, crop_X, crop_Y, crop_w, crop_h);

	cvtColor(open_cv_image, open_cv_image, COLOR_RGB2BGR);
	for (unsigned int i = 0; i < sick_points.size(); i++)
	{
		//cout<<sick_points[i].ipx<<" "<<sick_points[i].ipy<<endl;

		cv::circle(open_cv_image, cv::Point(sick_points[i].image_x,	sick_points[i].image_y), 1, cv::Scalar(0, 0, 255), 1);
	}


	cv::imshow("test", open_cv_image);
	cv::waitKey(10);


}


void
show_detections2(cv::Mat &rgb_image, vector<bbox_t> predictions, string window_name, string color)
{
    char confianca[25];
    int line_tickness = 1;
//    char frame_rate[25];
//
//    sprintf(frame_rate, "FPS = %.2f", fps);
//
//    //cv::putText(*rgb_image, frame_rate, cv::Point(10, 25), cv::FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    for (unsigned int i = 0; i < predictions.size(); i++)
    {

//		for (unsigned int j = 0; j < laser_points_in_camera_box_list[i].size(); j++)
//			cv::circle(*rgb_image, cv::Point(laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipx,
//					laser_points_in_camera_box_list[i][j].velodyne_points_in_cam.ipy), 1, cv::Scalar(0, 0, 255), 1);

        cv::Scalar object_color;

        sprintf(confianca, "%d  %.3f", predictions.at(i).obj_id, predictions.at(i).prob);

        int obj_id = predictions.at(i).obj_id;

        string obj_name;
        if (obj_names.size() > obj_id)
            obj_name = obj_names[obj_id];
//
        if (obj_name.compare("car") == 0)
        {
        	if (color.compare("red") == 0)
        	{
        		object_color = cv::Scalar(0, 0, 255);
        		line_tickness = 3;
        	}
        	if (color.compare("green") == 0)
        	{
        		object_color = cv::Scalar(0, 255, 0);
        		line_tickness = 1;
        	}
        	cv::rectangle(rgb_image,
        			cv::Point(predictions[i].x, predictions[i].y),
					cv::Point(predictions[i].x + predictions[i].w, predictions[i].y + predictions[i].h),
					object_color, line_tickness);

        }

    }

//    int thickness = -1;
//    int lineType = 8;
//    for (unsigned int i = 0; i < rddf_points.size(); i++)
//    {
//    	cv::circle(*rgb_image, cv::Point(rddf_points[i].x, rddf_points[i].y), 1.5, cv::Scalar(0, 255, 255), thickness, lineType);
//    }



    //cv::Mat resized_image(cv::Size(640, 480 - 480 * hood_removal_percentage), CV_8UC3);
    //cv::resize(rgb_image, resized_image, resized_image.size());
    //cv::resize(rgb_image, rgb_image, cv::Size(640, 364));
    cv::imshow(window_name, rgb_image);
    cv::waitKey(1);

    //resized_image.release();
}


void
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	vector<carmen_position_t> rddf_points_in_image;
	vector<carmen_position_t> rddf_points_in_image_full;
	vector<double> distances_of_rddf_from_car;
	double hood_removal_percentage = 0.2;
	carmen_velodyne_partial_scan_message velodyne_sync_with_cam;
	cv::Size size(320, 320);

    cv::Mat src_image = cv::Mat(cv::Size(image_msg->width, image_msg->height - image_msg->height * hood_removal_percentage), CV_8UC3);
    cv::Mat rgb_image = cv::Mat(cv::Size(image_msg->width, image_msg->height - image_msg->height * hood_removal_percentage), CV_8UC3);

    static double start_time = 0.0;
	double fps;

    if (camera_side == 0)
        memcpy(src_image.data, image_msg->raw_left, image_msg->image_size * sizeof(char) - image_msg->image_size * hood_removal_percentage * sizeof(char));
    else
        memcpy(src_image.data, image_msg->raw_right, image_msg->image_size * sizeof(char) - image_msg->image_size * hood_removal_percentage * sizeof(char));

    if (velodyne_vector.size() > 0)
        velodyne_sync_with_cam = find_velodyne_most_sync_with_cam(image_msg->timestamp); // TODO não faz sentido! Tem que sempre pegar a ultima msg do velodyne
    else
        return;

    cv::Mat src_image_copy = src_image.clone();

    cv::Mat pRoi = src_image_copy(cv::Rect(src_image_copy.cols * crop_x / 2.0, 0,
    		src_image_copy.cols - src_image_copy.cols * crop_x, src_image_copy.rows));
    src_image = pRoi;
    src_image_copy = src_image.clone();

    cv::cvtColor(src_image, rgb_image, cv::COLOR_RGB2BGR);

    cv::Mat rgb_image_copy = rgb_image.clone();
    or_image = rgb_image.clone();

    vector<bbox_t> bounding_boxes_of_slices_in_original_image;
    vector<cv::Scalar> colors;
    vector<cv::Mat> scene_slices;
    vector<t_transform_factor> transform_factor_of_slice_to_original_frame;
    if (strcmp(detection_type,"-ss") == 0)
    {
    	bounding_boxes_of_slices_in_original_image = darknet->detect(src_image, 0.2);
    	detections(bounding_boxes_of_slices_in_original_image, image_msg, velodyne_sync_with_cam, src_image, &rgb_image, start_time, fps, rddf_points_in_image, "Original Detection");
    	colors = get_slice_colors (1);
    }
    else if (strcmp(detection_type,"-cs") == 0)
    {
    	carmen_pose_3D_t car_pose = filter_pitch(pose);
    	tf::StampedTransform world_to_camera_pose = get_world_to_camera_transformation(&transformer, car_pose);

    	cv::Mat out;
    	out = rgb_image;
    	rddf_points_in_image = get_rddf_points_in_image(meters_spacement, distances_of_rddf_from_car, world_to_camera_pose, image_msg->width, image_msg->height);
    	rddf_points_in_image_full = get_rddf_points_in_image_full(world_to_camera_pose, image_msg->width, image_msg->height);

    	vector<cv::Mat> scene_slices_resized;
    	t_transform_factor t;
    	scene_slices.push_back(out);
    	t.scale_factor_x = 1;
    	t.scale_factor_y = 1;
    	t.translate_factor_x = 0;
    	t.translate_factor_y = 0;
    	transform_factor_of_slice_to_original_frame.push_back(t);
    	get_image_slices(scene_slices, transform_factor_of_slice_to_original_frame, out, rddf_points_in_image, distances_of_rddf_from_car);
    	vector<vector<bbox_t>> bounding_boxes_of_slices;


//    	char arr[50];
//    	char gt_path[200];
//    	strcpy(gt_path, groundtruth_path);
//    	sprintf(gt_path,"%s/%lf", gt_path, image_msg->timestamp);
//    	string str_gt_path (gt_path);
//    	string groundtruth_folder = str_gt_path + "-r.txt";
//
//    	cout<<groundtruth_folder<<endl;
//    	char classe[50];
//    	int x1, x2, y1, y2;
////    	FILE *f_groundtruth = fopen (groundtruth_folder.c_str(), "r");
////    	if (f_groundtruth != NULL)
////    		cout<<"Nao pude abrir o arquivo"<<endl;
////    	struct stat st;
////    	stat(groundtruth_folder.c_str(), &st);
////    	fscanf (f_groundtruth, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2);
//    	bbox_t gt;
//    	gt.x = x1;
//    	gt.y = y1;
//    	gt.w = x2 - x1;
//    	gt.h = y2 - y1;

    	cv::Mat crop0;
    	for (int i = 0; i < qtd_crops; i++)
    	{
    		vector<bbox_t> predictions;
//    		stringstream ss;
//    		ss << i;
//    		string name = "Foviated Detection" + ss.str();
    		predictions = get_predictions_of_slices(i, scene_slices[i]);
    		//cout<<"predictions "<<i<<" size: "<<predictions.size()<<endl;
//    		cv::Mat slice = scene_slices[i].clone();
////    		if (i==0){
////    			cv::rectangle(rgb_image,
////    					cv::Point(gt.x, gt.y),
////						cv::Point(gt.x + gt.w, gt.y + gt.h),
////						cvScalar(255,255,0), 2);
////    		}
//    		detections(predictions, image_msg, velodyne_sync_with_cam, src_image, &scene_slices[i], start_time, fps, rddf_points_in_image_full, name);
//    		show_detections2(slice, predictions, name, "red");
//    		if (i==0)
//    			crop0 = slice.clone();
    		bounding_boxes_of_slices.push_back(predictions);
    	}

    	bounding_boxes_of_slices_in_original_image = transform_bounding_boxes_of_slices(bounding_boxes_of_slices, transform_factor_of_slice_to_original_frame);

    	//show_detections2(crop0, bounding_boxes_of_slices_in_original_image, "Foviated Detection0", "green");

    	rgb_image = scene_slices[0];
    	src_image = scene_slices[0];
    	//cout<<"qtd of detections: "<<bounding_boxes_of_slices_in_original_image.size()<<endl;
    	detections(bounding_boxes_of_slices_in_original_image, image_msg, velodyne_sync_with_cam, src_image, &rgb_image, start_time, fps, rddf_points_in_image_full, "Foviated Detection");

    	colors = get_slice_colors (qtd_crops);

    	//printf("%lf-r.png\n", image_msg->timestamp);
  }

    save_detections(image_msg->timestamp, bounding_boxes_of_slices_in_original_image, rgb_image, scene_slices, colors,
    				transform_factor_of_slice_to_original_frame, rddf_points_in_image, rddf_points_in_image_full);
    //cout<<image_msg->timestamp<<"-r.png"<<endl;
//	publish_moving_objects_message(image_msg->timestamp);
}


void
velodyne_partial_scan_message_handler(carmen_velodyne_partial_scan_message *velodyne_message)
{
    velodyne_message_arrange = velodyne_message;

    carmen_velodyne_camera_calibration_arrange_velodyne_vertical_angles_to_true_position(velodyne_message_arrange);

    carmen_velodyne_partial_scan_message velodyne_copy;

    velodyne_copy.host = velodyne_message_arrange->host;
    velodyne_copy.number_of_32_laser_shots = velodyne_message_arrange->number_of_32_laser_shots;

    velodyne_copy.partial_scan = (carmen_velodyne_32_laser_shot *) malloc(
            sizeof(carmen_velodyne_32_laser_shot) * velodyne_message_arrange->number_of_32_laser_shots);

    memcpy(velodyne_copy.partial_scan, velodyne_message_arrange->partial_scan,
           sizeof(carmen_velodyne_32_laser_shot) * velodyne_message_arrange->number_of_32_laser_shots);

    velodyne_copy.timestamp = velodyne_message_arrange->timestamp;

    velodyne_vector.push_back(velodyne_copy);

    if (velodyne_vector.size() > maxPositions)
    {
        free(velodyne_vector.begin()->partial_scan);
        velodyne_vector.erase(velodyne_vector.begin());
    }
}


void
playback_command_handler(carmen_playback_command_message *command)
{
	//command_of_playback = command;
	cout<<command->cmd<<" "<<command->message<<" "<<command->offset<<" "<<command->speed<<endl;
}



void
carmen_laser_ldmrs_new_message_handler(carmen_laser_ldmrs_new_message* laser_message)
{
	sick_laser_message = laser_message;

	carmen_laser_ldmrs_new_message sick_copy;
	sick_copy.host = sick_laser_message->host;
	sick_copy.angle_ticks_per_rotation = sick_laser_message->angle_ticks_per_rotation;
	sick_copy.end_angle = sick_laser_message->end_angle;
	sick_copy.flags = sick_laser_message->flags;
	sick_copy.scan_end_time = sick_laser_message->scan_end_time;
	sick_copy.scan_number = sick_laser_message->scan_number;
	sick_copy.scan_points = sick_laser_message->scan_points;
	sick_copy.scan_start_time = sick_laser_message->scan_start_time;
	sick_copy.scanner_status = sick_laser_message->scanner_status;
	sick_copy.start_angle = sick_laser_message->start_angle;
	sick_copy.sync_phase_offset = sick_laser_message->sync_phase_offset;
	sick_copy.timestamp = sick_laser_message->timestamp;

	sick_copy.arraypoints = (carmen_laser_ldmrs_new_point *) malloc(
			sizeof(carmen_laser_ldmrs_new_point) * sick_laser_message->scan_points);

	memcpy(sick_copy.arraypoints, sick_laser_message->arraypoints,
			sizeof(carmen_laser_ldmrs_new_point) * sick_laser_message->scan_points);

	sick_vector.push_back(sick_copy);

	if (sick_vector.size() > MAX_POSITIONS)
	{
		free(sick_vector.begin()->arraypoints);
		sick_vector.erase(sick_vector.begin());
	}
}


void
localize_ackerman_globalpos_message_handler(carmen_localize_ackerman_globalpos_message *globalpos_message)
{
	pose = globalpos_message->pose;
    globalpos.theta = globalpos_message->globalpos.theta;
    globalpos.x = globalpos_message->globalpos.x;
    globalpos.y = globalpos_message->globalpos.y;

    //printf("Global pos: %lf X %lf Theta: %lf\n", globalpos.x, globalpos.y, globalpos.theta);
    //printf("Global pose: %lf X %lf X lf Theta: %lf Roll: %lf Pitch: %lf\n", pose.position.x, pose.position.y, pose.position.y, pose.orientation.yaw, pose.orientation.roll, pose.orientation.pitch);
}


void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        cvDestroyAllWindows();

        printf("Neural Object Detector 2: disconnected.\n");
        exit(0);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////
void
create_folders()
{
	stringstream ss;
	ss << meters_spacement;
	string str_log_name(log_name);
	char folder_name[100];
	char folder_image_name[100];
	int retorno;
	if (strcmp(detection_type,"-cs") == 0)
	{
		sprintf(folder_name, "%s_%.0lf_mts_detections/", log_name,meters_spacement);
		sprintf(folder_image_name, "%s_%.0lf_mts_images/", log_name,meters_spacement);
		str_folder_name = folder_name;
		str_folder_image_name = folder_image_name;
		string command;

		if (access(str_folder_name.c_str(), F_OK) != 0)
		{
			command = "mkdir " + str_folder_name;
			retorno = system(command.c_str());
		}

		if (access(str_folder_image_name.c_str(), F_OK) != 0)
		{
			command = "mkdir " + str_folder_image_name;
			retorno = system(command.c_str());

			command = "mkdir " + str_folder_image_name + "original/";
			str_folder_image_name_original = str_folder_image_name + "original/";
			retorno = system(command.c_str());

			command = "mkdir " + str_folder_image_name + "rddf_full/";
			str_folder_image_name_rddf_full = str_folder_image_name + "rddf_full/";
			retorno = system(command.c_str());

			command = "mkdir " + str_folder_image_name + "rddf_filtered/";
			str_folder_image_name_rddf_filtered = str_folder_image_name + "rddf_filtered/";
			retorno = system(command.c_str());

			command = "mkdir " + str_folder_image_name + "slices/";
			str_folder_image_name_slices = str_folder_image_name + "slices/";
			retorno = system(command.c_str());

			command = "mkdir " + str_folder_image_name + "slices_rddf_full/";
			str_folder_image_name_slices_rddf_full = str_folder_image_name + "slices_rddf_full/";
			retorno = system(command.c_str());

			command = "mkdir " + str_folder_image_name + "slices_rddf_filtered/";
			str_folder_image_name_slices_rddf_filtered = str_folder_image_name + "slices_rddf_filtered/";
			retorno = system(command.c_str());

			command = "mkdir " + str_folder_image_name + "slices_and_detection/";
			str_folder_image_name_slices_and_detection = str_folder_image_name + "slices_and_detection/";
			retorno = system(command.c_str());
		}
	}

	else if (strcmp(detection_type,"-ss") == 0)
	{
		sprintf(folder_name, "%s_detections/", log_name);
		str_folder_name = folder_name;
		string command;
		if (access(str_folder_name.c_str(), F_OK) != 0)
		{
			command = "mkdir " + str_folder_name;
			retorno = system(command.c_str());
		}

	}
}

void
subscribe_messages()
{
    carmen_bumblebee_basic_subscribe_stereoimage(camera, NULL, (carmen_handler_t) image_handler, CARMEN_SUBSCRIBE_LATEST);

    //carmen_subscribe_playback_info_message(NULL, (carmen_handler_t) playback_command_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_velodyne_subscribe_partial_scan_message(NULL, (carmen_handler_t) velodyne_partial_scan_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_laser_subscribe_ldmrs_new_message(NULL, (carmen_handler_t) carmen_laser_ldmrs_new_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_localize_ackerman_subscribe_globalpos_message(NULL, (carmen_handler_t) localize_ackerman_globalpos_message_handler, CARMEN_SUBSCRIBE_LATEST);

    //carmen_behavior_selector_subscribe_goal_list_message(NULL, (carmen_handler_t) behaviour_selector_goal_list_message_handler, CARMEN_SUBSCRIBE_LATEST);



    carmen_rddf_subscribe_annotation_message(NULL, (carmen_handler_t) rddf_annotation_message_handler, CARMEN_SUBSCRIBE_LATEST);

    carmen_subscribe_message((char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_NAME, (char *) CARMEN_BEHAVIOR_SELECTOR_ROAD_PROFILE_MESSAGE_FMT,
        			NULL, sizeof (carmen_behavior_selector_road_profile_message), (carmen_handler_t) rddf_handler, CARMEN_SUBSCRIBE_LATEST);
}


int
read_parameters(int argc, char **argv)
{
	camera = atoi(argv[1]);             // Define the camera to be used
	camera_side = atoi(argv[2]);        // 0 For left image 1 for right image
	char *meters;
	meters = argv[3];
	meters_spacement = atoi(meters);
	crops_tam = argv[4];
	qtd_crops = atoi(crops_tam);
	log_name = argv[5];
	groundtruth_path = argv[6];
	detection_type = argv[7];


    int num_items;

    char bumblebee_string[256];
    char camera_string[256];
    char bullbar_string[256];
    char sick_string[256];
    char velodyne_string[256];
    char sensor_board_string[256];



    sprintf(bumblebee_string, "%s%d", "bumblebee_basic", camera); // Geather the cameri ID
    sprintf(camera_string, "%s%d", "camera", camera);
    sprintf(bullbar_string, "%s", "front_bullbar");
    sprintf(sick_string, "%s", "laser_ldmrs");
    sprintf(velodyne_string, "%s", "velodyne");
    sprintf(sensor_board_string, "%s", "sensor_board_1");

    carmen_param_t param_list[] =
    {
		{bumblebee_string, (char*) "fx", CARMEN_PARAM_DOUBLE, &camera_parameters.fx_factor, 0, NULL },
		{bumblebee_string, (char*) "fy", CARMEN_PARAM_DOUBLE, &camera_parameters.fy_factor, 0, NULL },
		{bumblebee_string, (char*) "cu", CARMEN_PARAM_DOUBLE, &camera_parameters.cu_factor, 0, NULL },
		{bumblebee_string, (char*) "cv", CARMEN_PARAM_DOUBLE, &camera_parameters.cv_factor, 0, NULL },
		{bumblebee_string, (char*) "pixel_size", CARMEN_PARAM_DOUBLE, &camera_parameters.pixel_size, 0, NULL },

		{sensor_board_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &board_pose.position.x, 0, NULL },
		{sensor_board_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &board_pose.position.y, 0, NULL },
		{sensor_board_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &board_pose.position.z, 0, NULL },
		{sensor_board_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &board_pose.orientation.roll, 0, NULL },
		{sensor_board_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &board_pose.orientation.pitch, 0, NULL },
		{sensor_board_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &board_pose.orientation.yaw, 0, NULL },

		{camera_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &camera_pose.position.x, 0, NULL },
		{camera_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &camera_pose.position.y, 0, NULL },
		{camera_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &camera_pose.position.z, 0, NULL },
		{camera_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &camera_pose.orientation.roll, 0, NULL },
		{camera_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &camera_pose.orientation.pitch, 0, NULL },
		{camera_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &camera_pose.orientation.yaw, 0, NULL },

		{bullbar_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.x, 0, NULL },
		{bullbar_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.y, 0, NULL },
		{bullbar_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &bullbar_pose.position.z, 0, NULL },
		{bullbar_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.roll, 0, NULL },
		{bullbar_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.pitch, 0, NULL },
		{bullbar_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &bullbar_pose.orientation.yaw, 0, NULL },

		{sick_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &sick_pose.position.x, 0, NULL },
		{sick_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &sick_pose.position.y, 0, NULL },
		{sick_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &sick_pose.position.z, 0, NULL },
		{sick_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &sick_pose.orientation.roll, 0, NULL },
		{sick_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &sick_pose.orientation.pitch, 0, NULL },
		{sick_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &sick_pose.orientation.yaw, 0, NULL },

		{velodyne_string, (char*) "x",     CARMEN_PARAM_DOUBLE, &velodyne_pose.position.x, 0, NULL },
		{velodyne_string, (char*) "y",     CARMEN_PARAM_DOUBLE, &velodyne_pose.position.y, 0, NULL },
		{velodyne_string, (char*) "z",     CARMEN_PARAM_DOUBLE, &velodyne_pose.position.z, 0, NULL },
		{velodyne_string, (char*) "roll",  CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.roll, 0, NULL },
		{velodyne_string, (char*) "pitch", CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.pitch, 0, NULL },
		{velodyne_string, (char*) "yaw",   CARMEN_PARAM_DOUBLE, &velodyne_pose.orientation.yaw, 0, NULL }



		//LER SICK DO CARMEN INI
    };

    SampleFilter_init(&filter2);
    num_items = sizeof(param_list) / sizeof(param_list[0]);
    carmen_param_install_params(argc, argv, param_list, num_items);

    return 0;
}


int
main(int argc, char **argv)
{
	if ((argc != 8))
	        carmen_die("%s: Wrong number of parameters. neural_object_detector2 requires 2 parameter and received %d. \n Usage: %s <camera_number> <camera_side(0-left; 1-right)>"
	        		" <meters_spacement> <qtd_crops> <log_name> <groundtruth_path> <-cs for slices -ss without slices>\n",
	                   argv[0], argc - 1, argv[0]);

    int device_id = 0;

    string darknet_home = getenv("DARKNET_HOME");  // Get environment variable pointing path of darknet
    if (darknet_home.empty())
        printf("Cannot find darknet path. Check if you have correctly set DARKNET_HOME environment variable.\n");

    string cfg_filename = darknet_home + "/cfg/neural_object_detector_yolo.cfg";
    string weight_filename = darknet_home + "/yolo.weights";
    string class_names_file = darknet_home + "/data/coco.names";
    obj_names = objects_names_from_file(class_names_file);

    darknet = new Detector(cfg_filename, weight_filename, device_id);
    carmen_test_alloc(darknet);

//#ifdef SHOW_DETECTIONS
//    cv::namedWindow("Neural Object Detector", cv::WINDOW_AUTOSIZE);
//#endif

    setlocale(LC_ALL, "C");

    carmen_ipc_initialize(argc, argv);

    signal(SIGINT, shutdown_module);

    read_parameters(argc, argv);

    create_folders();

    initialize_transformations(board_pose, camera_pose, &transformer);
    initialize_sick_transformations(board_pose, camera_pose, bullbar_pose, sick_pose, &transformer_sick);

    subscribe_messages();

    //printf("%lf %lf\n", rddf_msg.x, rddf_msg.y);

    carmen_ipc_dispatch();

    return 0;
}

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


typedef struct
{
	int i;
	carmen_position_t camera_pos;
	carmen_position_t rddf_pos;
	vector <carmen_position_t> rddf_list;
}debug_infos;

vector<debug_infos> closest_rddf;


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

carmen_behavior_selector_road_profile_message goal_list_message;

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

    }

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
				vector<carmen_position_t> rddf_points_in_image_filtered, vector<carmen_position_t> rddf_points_in_image_full, vector<vector<bbox_t>> bounding_boxes_of_detections)
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

//		before_first_file = false;
//		acessessing = true;
//		FILE *f_groundtruth = fopen (groundtruth_folder.c_str(), "r");
//		struct stat st;
//		stat(groundtruth_folder.c_str(), &st);
//		int size = st.st_size;
//		if (size == 0)
//		{
//			FILE *f_detection = fopen (detections_folder.c_str(), "w");
//			fclose (f_detection);
//			fclose (f_groundtruth);
//		}
//		else
//		{
//			char classe [10];
//			float x1, y1, x2, y2;
//
//			vector<bbox_t> bounding_boxes_of_gt;
//			bbox_t gt;
//			while (fscanf (f_groundtruth, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2) != EOF)
//			{
//		    	gt.x = (int)x1;
//		    	gt.y = (int)y1;
//		    	gt.w = (int)(x2 - x1);
//		    	gt.h = (int)(y2 - y1);
//		    	bounding_boxes_of_gt.push_back(gt);
//			}
//
//			FILE *f_detection = fopen (detections_folder.c_str(), "w");
////			fprintf (f_detection, "Camera Pos: %lf X %lf\nInitial RDDF index: %d (%lf X %lf) \n", closest_rddf[0].camera_pos.x, closest_rddf[0].camera_pos.y, closest_rddf[0].i, closest_rddf[0].rddf_pos.x, closest_rddf[0].rddf_pos.y);
//
////			for (int i = 0; i < bounding_boxes_of_detections.size(); i++)
////			{
////				stringstream ss;
////				ss << i;
////				string im_name = str_arr + "-r.png";
////				for (int j = 0; j < bounding_boxes_of_detections[i].size(); j++)
////				{
////
////					bbox_t b = bounding_boxes_of_detections[i][j];
////
////					if (i == 0)
////						fprintf (f_detection, "Crop %d %s %f %.2f %.2f %.2f %.2f\n", i, "car", b.prob, (float)b.x, (float)b.y, (float)(b.x + b.w), (float)(b.y + b.h));
////					else
////						fprintf (f_detection, "Crop %d pos in image: %lf X %lf (RDDF Index: %d (%lf X %lf)) %s %f %.2f %.2f %.2f %.2f\n", i, rddf_points_in_image_filtered[i].x, rddf_points_in_image_filtered[i].y, closest_rddf[i+1].i, closest_rddf[i+1].rddf_pos.x, closest_rddf[i+1].rddf_pos.y, "car", b.prob, (float)b.x, (float)b.y, (float)(b.x + b.w), (float)(b.y + b.h));
////					if (i == 0)
////						cv::rectangle(or_image,
////								cv::Point(b.x, b.y),
////								cv::Point(b.x + b.w, b.y + b.h),
////								cvScalar (0,0,255), 1);
////
////					else if (i == 1)
////					{
////						b.x = b.x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
////						b.y = b.y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;
////						cv::rectangle(or_image,
////								cv::Point(b.x, b.y),
////								cv::Point(b.x + b.w, b.y + b.h),
////								cvScalar (0,255,0), 1);
////					}
////
////					else if (i == 2)
////					{
////						b.x = b.x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
////						b.y = b.y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;
////						cv::rectangle(or_image,
////								cv::Point(b.x, b.y),
////								cv::Point(b.x + b.w, b.y + b.h),
////								cvScalar (255,0,0), 1);
////					}
////
////					else if (i == 3)
////					{
////						b.x = b.x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
////						b.y = b.y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;
////						cv::rectangle(or_image,
////								cv::Point(b.x, b.y),
////								cv::Point(b.x + b.w, b.y + b.h),
////								cvScalar (255,255,0), 1);
////					}
////
////					else if (i == 4)
////					{
////						b.x = b.x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
////						b.y = b.y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;
////						cv::rectangle(or_image,
////								cv::Point(b.x, b.y),
////								cv::Point(b.x + b.w, b.y + b.h),
////								cvScalar (255,0,255), 1);
////					}
////
////				}
////				imwrite(im_name, or_image);
////			}
//
//			for (int i = 0; i < bounding_boxes_of_gt.size(); i++)
//			{
//				cv::Rect rect_A;
//				rect_A.x = bounding_boxes_of_gt[i].x; rect_A.y = bounding_boxes_of_gt[i].y;
//				rect_A.width = bounding_boxes_of_gt[i].w; rect_A.height = bounding_boxes_of_gt[i].h;
//				for (int j = 0; j < bounding_boxes_of_slices_in_original_image.size(); j++)
//				{
//					//cout<<"\t"<<i<<endl;
//					bbox_t b = bounding_boxes_of_slices_in_original_image[j];
//					cv::Rect rect_B;
//					rect_B.x = (int)b.x; rect_B.y = (int)b.y;
//					rect_B.width = (int)b.w; rect_B.height = b.h;
//					//					cv::Point l1, r1, l2, r2;
//					//					l1.x = (int)x1; l1.y = (int)y1; //top left
//					//					r1.x = (int)x2; r1.y = (int)y2; //right botton of groundtruth bbox
//					//					l2.x = (int)b.x; l2.y = (int)b.y; //top left
//					//					r2.x = (int)b.x + b.w; r2.y = (int)b.y + b.h; //right botton of detection
//					//cout<<x1<<" "<<x1<<" "
//					//cout<<classe<<" "<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<endl;
//					//if(rectangles_intersects(l1, r1, l2, r2))
//					if(rectangles_intersects(rect_A, rect_B) || rectangles_intersects(rect_B, rect_A))
//					{
//						//cout<<"\t"<<i<<" "<<obj_names[obj_id]<<endl;
//						fprintf (f_detection, "%s %f %.2f %.2f %.2f %.2f\n", "car", b.prob, (float)b.x, (float)b.y, (float)(b.x + b.w), (float)(b.y + b.h));
//					}
//
//				}
//			}


//
//			fclose (f_detection);
//			fclose (f_groundtruth);
//			closest_rddf.clear();

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
										cv::Point l1, r1, l2, r2;
										l1.x = (int)x1; l1.y = (int)y1; //top left
										r1.x = (int)x2; r1.y = (int)y2; //right botton of groundtruth bbox
										l2.x = (int)b.x; l2.y = (int)b.y; //top left
										r2.x = (int)b.x + b.w; r2.y = (int)b.y + b.h; //right botton of detection
					//cout<<x1<<" "<<x1<<" "
					//cout<<classe<<" "<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<endl;
					//if(rectangles_intersects(l1, r1, l2, r2))
//										calc_percentage_of_rectangles_intersection(l1, r1, l2, r2) > 50
//					if(rectangles_intersects(rect_A, rect_B))
//					{
						//cout<<"\t"<<i<<" "<<obj_names[obj_id]<<endl;
						fprintf (f_detection, "%s %f %.2f %.2f %.2f %.2f\n", "car", b.prob, (float)b.x, (float)b.y, (float)(b.x + b.w), (float)(b.y + b.h));
//					}
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
intersectionOverUnion(bbox_t box1, bbox_t box2)
{
	// https://github.com/lukaswals/cpp-iout/blob/master/cppIOUT/IOUT.cpp
	float minx1 = box1.x;
	float maxx1 = box1.x + box1.w;
	float miny1 = box1.y;
	float maxy1 = box1.y+ box1.h;

	float minx2 = box2.x;
	float maxx2 = box2.x + box2.w;
	float miny2 = box2.y;
	float maxy2 = box2.y + box2.h;

	if (minx1 > maxx2 || maxx1 < minx2 || miny1 > maxy2 || maxy1 < miny2)
		return (0.0);
	else
	{
		float dx = std::min(maxx2, maxx1) - std::max(minx2, minx1);
		float dy = std::min(maxy2, maxy1) - std::max(miny2, miny1);
		float area1 = (maxx1 - minx1)*(maxy1 - miny1);
		float area2 = (maxx2 - minx2)*(maxy2 - miny2);
		float inter = dx*dy; // Intersection
		float uni = area1 + area2 - inter; // Union
		float IoU = inter / uni;

		return (IoU);
	}
}


float
calc_percentage_of_rectangles_intersection(cv::Point l1, cv::Point r1, cv::Point l2, cv::Point r2)
{
	bbox_t box1, box2;

	box1.x = l1.x;
	box1.y = l1.y;
	box1.w = r1.x - l1.x;
	box1.h = r1.y - l1.y;

	box2.x = l2.x;
	box2.y = l2.y;
	box2.w = r2.x - l2.x;
	box2.h = r2.y - l2.y;

	return (100.0 * intersectionOverUnion(box1, box2));
}


float
calc_percentage_of_rectangles_intersection_old(cv::Point l1, cv::Point r1, cv::Point l2, cv::Point r2)
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
transform_bounding_boxes_of_slices (vector< vector<bbox_t> > bounding_boxes_of_slices, vector<t_transform_factor> transform_factor_of_slice_to_original_frame)
{
	cv::Mat img;
	img = or_image.clone();
	vector<bbox_t> bboxes;
	bbox_t b;
	bool intersects_with_bboxes = false;
	bool rect_dont_intersects = false;
	bool intersect;
	for (int i = 0; i < bounding_boxes_of_slices.size(); i++)
	{
		for (int j = 0; j < bounding_boxes_of_slices[i].size(); j++)
		{
			intersect = false;

			int obj_id = bounding_boxes_of_slices[i][j].obj_id;
			string obj_name;
			if (obj_names.size() > obj_id)
				obj_name = obj_names[obj_id];

			if (obj_name.compare("car") == 0)
			{
				b = bounding_boxes_of_slices[i][j];
				b.x = bounding_boxes_of_slices[i][j].x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
				b.y = bounding_boxes_of_slices[i][j].y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;

				cv::Point l1; //top left
				l1.x = b.x;
				l1.y = b.y;
				cv::Point r1; //bottom right
				r1.x = b.x + b.w;
				r1.y = b.y + b.h;

				int k = 0;
				while (k < bboxes.size())
				{
					cv::Point l2;
					l2.x = bboxes[k].x;
					l2.y = bboxes[k].y;
					cv::Point r2;
					r2.x = bboxes[k].x + bboxes[k].w;
					r2.y = bboxes[k].y + bboxes[k].h;

					float percentage_of_intersection_between_bboxes = calc_percentage_of_rectangles_intersection(l1, r1, l2, r2);

					if (percentage_of_intersection_between_bboxes > 50.0)
					{
						if (b.prob > bboxes[k].prob)
						{
							bboxes.erase(bboxes.begin() + k);
							intersect = false;
							k--;
						}
						else
							intersect = true;
					}
					k++;
				}

				if (!intersect)
					bboxes.push_back(b);
			}
		}
	}

	return (bboxes);
}
//for (int k = 0; k < bboxes.size(); k++)
//{
//	cv::Point l2;
//	l2.x = bboxes[k].x;
//	l2.y = bboxes[k].y;
//	cv::Point r2;
//	r2.x = bboxes[k].x + bboxes[k].w;
//	r2.y = bboxes[k].y + bboxes[k].h;
//
//	float percentage_of_intersection_between_bboxes = calc_percentage_of_rectangles_intersection(l1, r1, l2, r2);
//
//	if (percentage_of_intersection_between_bboxes > 50.0)
//	{
//
//		if (b.prob > bboxes[k].prob)
//			bboxes[k] = b;
//		intersect = true;
//	}
//}


vector<bbox_t>
get_predictions_of_slices(int i, cv::Mat image)
{
	vector<bbox_t> predictions;
	predictions = darknet->detect(image, 0.2);  // Arguments (img, threshold)
	return (predictions);
}


void
get_image_slices(vector<cv::Mat> &scene_slices, vector<t_transform_factor> &transform_factor_of_slice_to_original_frame,
		cv::Mat image, vector<carmen_position_t> rddf_points_in_image_filtered,
		vector<double> distances_of_rddf_from_car)
{
	cv::Mat im;
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
	double scale = image_size_y * 0.75;
	top_left_point.x = static_cast<double>(rddf_points_in_image_filtered[0].x) - (image_size_x/2);
	top_left_point.y = static_cast<double>(rddf_points_in_image_filtered[0].y)-scale;
//	cout<<top_left_point.x<<" "<<top_left_point.y<<endl;
	cv::Rect rec(top_left_point.x, top_left_point.y, image_size_x, image_size_y);
	if (check_rect_inside_image(rec, image))
	{
		roi = image(rec);
		im = roi.clone();
//		imshow("test", im);
//		cv::waitKey(0);
		mult_scale_x += double(scene_slices[0].cols) / double(im.cols);
		mult_scale_y += double(scene_slices[0].rows) / double(im.rows);
		t.scale_factor_x = mult_scale_x;
		t.scale_factor_y = mult_scale_y;
		sum_transform_x += top_left_point.x;
		sum_transform_y += top_left_point.y;
		t.translate_factor_x = sum_transform_x;
		t.translate_factor_y = sum_transform_y;
//		cout<<t.translate_factor_x<<" "<<t.translate_factor_y<<endl;
		scene_slices.push_back(im);
		transform_factor_of_slice_to_original_frame.push_back(t);
	}

	//cout<<rddf_points_in_image_filtered.size()<<" "<<distances_of_rddf_from_car.size()<<endl;
	for (int i = 1; i < rddf_points_in_image_filtered.size(); i++)
	{
		image_size_x = (scene_slices[0].cols * first_crop_percentage) / (i + 1);
		image_size_y = (scene_slices[0].rows * first_crop_percentage) / (i + 1);
		if (image_size_x > 1)//
		{
			double scale = image_size_y * (0.75);
			top_left_point.x = static_cast<double>(rddf_points_in_image_filtered[i].x) - (image_size_x/2);
			top_left_point.y = static_cast<double>(rddf_points_in_image_filtered[i].y) - scale;
			cv::Rect rec(top_left_point.x, top_left_point.y, image_size_x, image_size_y);
			if (check_rect_inside_image(rec, image))
			{
				roi = image(rec);
				im = roi.clone();
//				imshow("test", im);
//						cv::waitKey(0);
				mult_scale_x = double(scene_slices[0].cols) / double(im.cols);
				mult_scale_y = double(scene_slices[0].rows) / double(im.rows);
				t.scale_factor_x = mult_scale_x;
				t.scale_factor_y = mult_scale_y;
				sum_transform_x = top_left_point.x;
				sum_transform_y = top_left_point.y;
				t.translate_factor_x = sum_transform_x;
				t.translate_factor_y = sum_transform_y;
//				cout<<t.translate_factor_x<<" "<<t.translate_factor_y<<endl;
				scene_slices.push_back(im);
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
	vector<carmen_position_t> rddf_points_in_image_full;
	double distance, last_distance;
	for(int i = 0; i < last_rddf_poses.number_of_poses; i++)
	{
		p = convert_rddf_pose_to_point_in_image (last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, 0.0, world_to_camera_pose, camera_parameters, img_width, img_height);
		rddf_points_in_image_full.push_back(p);
	}
	return (rddf_points_in_image_full);
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
//	*camera_pose_x = 7756770.756474;
//	*camera_pose_y = -363659.410522;

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
	vector<carmen_position_t> rddf_points_in_image_filtered;
	debug_infos d;

	int inicial_rddf_index;
	double camera_pose_x, camera_pose_y;
	inicial_rddf_index = get_closest_rddf_index(&camera_pose_x, &camera_pose_y);

	d.i = inicial_rddf_index;
	d.rddf_pos.x = last_rddf_poses.poses[inicial_rddf_index].x;
	d.rddf_pos.y = last_rddf_poses.poses[inicial_rddf_index].y;
	d.camera_pos.x = camera_pose_x;
	d.camera_pos.y = camera_pose_y;
	closest_rddf.push_back(d);

	double distance = 0.0, distance_ant = 0.0;
	double distance_accum = meters_spacement;
//
//	for (int i = 0; i < last_rddf_poses.number_of_poses; i++)
//	{
//		d.rddf_pos.x = last_rddf_poses.poses[i].x;
//		d.rddf_pos.y = last_rddf_poses.poses[i].y;
//		d.rddf_list.push_back(d);
//	}

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

			d.i = closest_point;
			d.rddf_pos.x = last_rddf_poses.poses[closest_point].x;
			d.rddf_pos.y = last_rddf_poses.poses[closest_point].y;
			closest_rddf.push_back(d);

			p = convert_rddf_pose_to_point_in_image (last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, 0.0, world_to_camera_pose, camera_parameters, img_width, img_height);
			distances_of_rddf_from_car.push_back(distance);
			rddf_points_in_image_filtered.push_back(p);
			distance_ant = distance;
		}
	}

	return (rddf_points_in_image_filtered);
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


void
show_detections_alberto(vector<t_transform_factor> transform_factor_of_slice_to_original_frame, vector<cv::Mat> scene_slices,
		vector<vector<bbox_t>> bounding_boxes_of_slices, vector<bbox_t> predictions,
		vector<carmen_position_t> rddf_points_in_image_filtered, double image_timestamp)
{
	printf("******************************************\n");
	printf("Timestamp %lf:\n\n", image_timestamp);
    char confianca[25];
    int line_tickness = 1;
//    char frame_rate[25];
//
//    sprintf(frame_rate, "FPS = %.2f", fps);
//
//    //cv::putText(*rgb_image, frame_rate, cv::Point(10, 25), cv::FONT_HERSHEY_PLAIN, 2, cvScalar(0, 255, 0), 2);

    char arr[50];
    string str_arr;
    char gt_path[200];
    strcpy(gt_path, groundtruth_path);
    sprintf(gt_path,"%s/%lf", gt_path, image_timestamp);
    string str_gt_path (gt_path);
    string groundtruth_folder = str_gt_path + "-r.txt";
    int thickness = -1;
    int lineType = 8;

    bbox_t gt;
    char classe[50];
    float x1, x2, y1, y2;
    if (access(groundtruth_folder.c_str(), F_OK) == 0)
    {
    	//cout<<groundtruth_folder<<" show"<<endl;
    	FILE *f;
    	f = fopen (groundtruth_folder.c_str(), "r");
    	//int yy = fscanf (f, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2);
    	//yy = yy;

    	while (fscanf (f, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2) != EOF)
    	{
    		gt.x = (int)x1;
    		gt.y = (int)y1;
    		gt.w = (int)(x2 - x1);
    		gt.h = (int)(y2 - y1);
    		cv::rectangle(scene_slices[0],
    				cv::Point(gt.x, gt.y),
					cv::Point(gt.x + gt.w, gt.y + gt.h),
					Scalar(0, 255, 0), 3);
    	}

    }
    else
    	exit(0);

    string name;
    for (int i = 0; i < qtd_crops; i++)
    {
    	cv::Mat image;
		stringstream ss;
//		char image_ts[40];
//		sprintf(image_ts,"%lf_",image_timestamp);
//		string im_ts (image_ts);
//		name = "Foveated Detection" + im_ts;
		ss << i;
		name = "Foveated Detection" + ss.str();

    	for (int j = 0; j < bounding_boxes_of_slices[i].size(); j++)
    	{
    		//cout<<bounding_boxes_of_slices[i].size()<<endl;

    		bbox_t b = bounding_boxes_of_slices[i][j];
    		bbox_t b_print = bounding_boxes_of_slices[i][j];
    		b_print.x = b_print.x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
    		b_print.y = b_print.y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;


    		cv::Scalar object_color;

    		//sprintf(confianca, "%d  %.3f", predictions.at(i).obj_id, predictions.at(i).prob);

    		int obj_id = bounding_boxes_of_slices[i][j].obj_id;

    		string obj_name;
    		if (obj_names.size() > obj_id)
    			obj_name = obj_names[obj_id];

    		//
			if (obj_name.compare("car") == 0)
			{
					object_color = cv::Scalar(0, 0, 255);
					line_tickness = 2;

//				image = scene_slices[i].clone();

					cv::rectangle(scene_slices[i],
							cv::Point(b.x, b.y),
							cv::Point(b.x + b.w, b.y + b.h),
							object_color, line_tickness);

				cout<<"Bboxes slice "<<i<<":"<<endl;
				printf("\tx1: %d, y1: %d, x2: %d, y2: %d, w: %d, h: %d - > %0.4f\n",
						b_print.x, b_print.y, b_print.x + b_print.w, b_print.x + b_print.w, b_print.w, b_print.h, b_print.prob);
			}

    	}
    	float iou;
    	float iou2;
    	if(i == qtd_crops-1)
    	{

    		cout<<endl<<endl<<"Groundtruth:"<<endl;
    		printf("\tx1: %d, y1: %d, x2: %d, y2: %d, w: %d, h: %d\n", gt.x, gt.y, gt.x + gt.w, gt.x + gt.w, gt.w, gt.h);
    		cout<<endl;

    		for (int k = 0; k < predictions.size(); k++)
    		{
    			bbox_t det;
    			det = predictions[k];
    			cv::Rect rect_A;
    			cv::Rect rect_B;
    			//bbox_t det;
    			det = predictions[k];
    			rect_A.x = (int)det.x; rect_A.y = (int)det.y;
    			rect_A.width = (int)det.w; rect_B.height = det.h;

    			cv::Point l1;
    			l1.x = det.x;
    			l1.y = det.y;
    			cv::Point r1;
    			r1.x = det.x + det.w;
    			r1.y = det.y + det.h;

    			rect_B.x = (int)gt.x; rect_B.y = (int)gt.x;
    			rect_B.width = (int)gt.w; rect_B.height = gt.h;
    			cv::Point l2;
    			l2.x = gt.x;
    			l2.y = gt.y;
    			cv::Point r2;
    			r2.x = gt.x + gt.w;
    			r2.y = gt.y + gt.h;

    			iou = calc_percentage_of_rectangles_intersection (l1, r1, l2, r2);
    			if (iou > 50)
    			{
    				cout<<"Filtered:"<<endl;
    				printf("\tx1: %d, y1: %d, x2: %d, y2: %d, w: %d, h: %d -> %0.4f  IOU: %f\n", det.x, det.y, det.x + det.w, det.x + det.w, det.w, det.h, det.prob, iou);
    			}
    			else
    			{
    				cout<<"Filtered:"<<endl;
    				printf("\tx1: %d, y1: %d, x2: %d, y2: %d, w: %d, h: %d -> %0.4f  IOU: %f ****** \n", det.x, det.y, det.x + det.w, det.x + det.w, det.w, det.h, det.prob, iou);
    			}


    		}
    	}

    	if (i == 0)
    	{
    		for (int l = 0; l < rddf_points_in_image_filtered.size(); l++)
    		{

    			cv::circle(scene_slices[0], cv::Point(rddf_points_in_image_filtered[l].x, rddf_points_in_image_filtered[l].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
    		}
    		for (int k = 0; k < predictions.size(); k++)
    		{
    			bbox_t det;
    			det = predictions[k];
    			float point_middle_det_x;
    			float pos_middle_det_x;
    			int is_in_rddf_filtered = 0;
    			cv::Rect rect_A;
    			cv::Rect rect_B;
    			//bbox_t det;
    			det = predictions[k];
    			rect_A.x = (int)det.x; rect_A.y = (int)det.y;
    			rect_A.width = (int)det.w; rect_B.height = det.h;

    			cv::Point l1;
    			l1.x = det.x;
    			l1.y = det.y;
    			cv::Point r1;
    			r1.x = det.x + det.w;
    			r1.y = det.y + det.h;


    			point_middle_det_x = (r1.x - l1.x) / 2;
    			pos_middle_det_x = l1.x + point_middle_det_x;

    			rect_B.x = (int)gt.x; rect_B.y = (int)gt.x;
    			rect_B.width = (int)gt.w; rect_B.height = gt.h;
    			cv::Point l2;
    			l2.x = gt.x;
    			l2.y = gt.y;
    			cv::Point r2;
    			r2.x = gt.x + gt.w;
    			r2.y = gt.y + gt.h;

    			iou = calc_percentage_of_rectangles_intersection (l1, r1, l2, r2);

//    			if (iou > 50)
//    				cv::rectangle(scene_slices[0],
//    						cv::Point(predictions[k].x, predictions[k].y),
//							cv::Point(predictions[k].x + predictions[k].w, predictions[k].y + predictions[k].h),
//							Scalar(255, 255, 0), 3);


    			for (int l = 0; l < rddf_points_in_image_filtered.size(); l++)
    			{
    				if ((pos_middle_det_x > (rddf_points_in_image_filtered[l].x - 150)) && (pos_middle_det_x < (rddf_points_in_image_filtered[l].x + 150)))
    					is_in_rddf_filtered++;
    			}

    			//if ((is_in_rddf_filtered == rddf_points_in_image_filtered.size()))
    				//					if (iou > 50)
    				cv::rectangle(scene_slices[0],
    						cv::Point(predictions[k].x, predictions[k].y),
							cv::Point(predictions[k].x + predictions[k].w, predictions[k].y + predictions[k].h),
							Scalar(0, 255, 255), 3);

    		}

    	}
    	cv:Mat aux_img = scene_slices[i];
    	if (i == 0)
    		cv::resize(aux_img, aux_img, Size(1152, 691));
    	else
    		cv::resize(aux_img, aux_img, Size(384, 230));
    	cv::imshow(name, aux_img);
    	cv::waitKey(1);
    }
    printf("******************************************\n\n\n\n\n\n");
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
image_handler(carmen_bumblebee_basic_stereoimage_message *image_msg)
{
	vector<carmen_position_t> rddf_points_in_image_filtered;
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
    	detections(bounding_boxes_of_slices_in_original_image, image_msg, velodyne_sync_with_cam, src_image, &rgb_image, start_time, fps, rddf_points_in_image_filtered, "Original Detection");
    	colors = get_slice_colors (1);
    }
    else if (strcmp(detection_type,"-cs") == 0)
    {
    	carmen_pose_3D_t car_pose = filter_pitch(pose);
    	tf::StampedTransform world_to_camera_pose = get_world_to_camera_transformation(&transformer, car_pose);

    	cv::Mat out;
    	out = rgb_image;
    	rddf_points_in_image_filtered = get_rddf_points_in_image(meters_spacement, distances_of_rddf_from_car, world_to_camera_pose, image_msg->width, image_msg->height);
    	rddf_points_in_image_full = get_rddf_points_in_image_full(world_to_camera_pose, image_msg->width, image_msg->height);

    	vector<cv::Mat> scene_slices_resized;
    	t_transform_factor t;
    	scene_slices.push_back(out);
    	t.scale_factor_x = 1;
    	t.scale_factor_y = 1;
    	t.translate_factor_x = 0;
    	t.translate_factor_y = 0;
    	transform_factor_of_slice_to_original_frame.push_back(t);
    	get_image_slices(scene_slices, transform_factor_of_slice_to_original_frame, out, rddf_points_in_image_filtered, distances_of_rddf_from_car);
    	vector<vector<bbox_t>> bounding_boxes_of_slices;


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

        char arr[50];
        char gt_path[200];
        strcpy(gt_path, groundtruth_path);
        sprintf(gt_path,"%s/%lf", gt_path, image_msg->timestamp);
        string str_gt_path (gt_path);
        string groundtruth_folder = str_gt_path + "-r.txt";

        if (access(groundtruth_folder.c_str(), F_OK) == 0)
        	show_detections_alberto(transform_factor_of_slice_to_original_frame ,scene_slices, bounding_boxes_of_slices, bounding_boxes_of_slices_in_original_image,
        			rddf_points_in_image_filtered, image_msg->timestamp);

    	rgb_image = scene_slices[0];
    	src_image = scene_slices[0];
    	//cout<<"qtd of detections: "<<bounding_boxes_of_slices_in_original_image.size()<<endl;
//    	detections(bounding_boxes_of_slices_in_original_image, image_msg, velodyne_sync_with_cam, src_image, &rgb_image, start_time, fps, rddf_points_in_image_full, "Foviated Detection");

    	colors = get_slice_colors (qtd_crops);

    	//printf("%lf-r.png\n", image_msg->timestamp);
//    	save_detections(image_msg->timestamp, bounding_boxes_of_slices_in_original_image, rgb_image, scene_slices, colors,
//    	    				transform_factor_of_slice_to_original_frame, rddf_points_in_image_filtered, rddf_points_in_image_full, bounding_boxes_of_slices);
  }


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

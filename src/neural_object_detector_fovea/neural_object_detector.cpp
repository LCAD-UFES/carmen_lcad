#include "neural_object_detector.hpp"


carmen_vector_3D_t
translate_point(carmen_vector_3D_t point, carmen_vector_3D_t offset)
{
	point.x += offset.x;
	point.y += offset.y;
	point.z += offset.z;
	return (point);
}



carmen_vector_3D_t
rotate_point(carmen_vector_3D_t point, double theta)
{
	carmen_vector_3D_t p;
	p.x = point.x * cos(theta) - point.y * sin(theta);
	p.y = point.x * sin(theta) + point.y * cos(theta);
	p.z = point.z;
	return (p);
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


void
get_image_crops(vector<cv::Mat> &scene_crops, vector<t_transform_factor> &transform_factor_of_slice_to_original_frame,
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
	int image_size_x = scene_crops[0].cols * first_crop_percentage;
	int image_size_y = scene_crops[0].rows * first_crop_percentage;
	double scale = image_size_y * 0.75;
	top_left_point.x = static_cast<double>(rddf_points_in_image_filtered[0].x) - (image_size_x/2);
	top_left_point.y = static_cast<double>(rddf_points_in_image_filtered[0].y)-scale;
	cv::Rect rec(top_left_point.x, top_left_point.y, image_size_x, image_size_y);
	if (check_rect_inside_image(rec, image))
	{
		roi = image(rec);
		im = roi.clone();
		mult_scale_x += double(scene_crops[0].cols) / double(im.cols);
		mult_scale_y += double(scene_crops[0].rows) / double(im.rows);
		t.scale_factor_x = mult_scale_x;
		t.scale_factor_y = mult_scale_y;
		sum_transform_x += top_left_point.x;
		sum_transform_y += top_left_point.y;
		t.translate_factor_x = sum_transform_x;
		t.translate_factor_y = sum_transform_y;
		scene_crops.push_back(im);
		transform_factor_of_slice_to_original_frame.push_back(t);
	}

	for (int i = 1; i < rddf_points_in_image_filtered.size(); i++)
	{
		image_size_x = (scene_crops[0].cols * first_crop_percentage) / (i + 1);
		image_size_y = (scene_crops[0].rows * first_crop_percentage) / (i + 1);
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
				mult_scale_x = double(scene_crops[0].cols) / double(im.cols);
				mult_scale_y = double(scene_crops[0].rows) / double(im.rows);
				t.scale_factor_x = mult_scale_x;
				t.scale_factor_y = mult_scale_y;
				sum_transform_x = top_left_point.x;
				sum_transform_y = top_left_point.y;
				t.translate_factor_x = sum_transform_x;
				t.translate_factor_y = sum_transform_y;
				scene_crops.push_back(im);
				transform_factor_of_slice_to_original_frame.push_back(t);
			}

		}

	}
}


vector<carmen_position_t>
get_rddf_points_in_image_full (tf::StampedTransform world_to_camera_pose,
		carmen_behavior_selector_road_profile_message last_rddf_poses, carmen_camera_parameters camera_parameters,
		int img_width, int img_height)
{
	carmen_position_t p;
	vector<carmen_position_t> rddf_points_in_image;
	double distance, last_distance;
	for(int i = 0; i < last_rddf_poses.number_of_poses; i++)
	{
		p = convert_rddf_pose_to_point_in_image (last_rddf_poses.poses[i].x, last_rddf_poses.poses[i].y, 0.0,
				world_to_camera_pose, camera_parameters, img_width, img_height);
		rddf_points_in_image.push_back(p);
	}
	return (rddf_points_in_image);
}


double
euclidean_distance (double x1, double x2, double y1, double y2)
{
	return ( sqrt(pow(x2-x1,2) + pow(y2-y1,2)) );
}


void
carmen_translate_2d(double *x, double *y, double offset_x, double offset_y)
{
	*x += offset_x;
	*y += offset_y;
}


int
get_closest_rddf_index(double *camera_pose_x, double *camera_pose_y, carmen_pose_3D_t camera_pose, carmen_pose_3D_t board_pose,
		carmen_point_t globalpos, carmen_behavior_selector_road_profile_message last_rddf_poses)
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
get_rddf_points_in_image_filtered_by_meters_spacement(double meters_spacement, vector<double> &distances_of_rddf_from_car, tf::StampedTransform world_to_camera_pose,
		carmen_pose_3D_t camera_pose, carmen_pose_3D_t board_pose, carmen_point_t globalpos,
		carmen_behavior_selector_road_profile_message last_rddf_poses, vector<debug_infos> closest_rddf, carmen_camera_parameters camera_parameters,
		int img_width, int img_height)
{

	carmen_position_t p;
	vector<carmen_position_t> rddf_points_in_image_filtered;
	debug_infos d;

	int inicial_rddf_index;
	double camera_pose_x, camera_pose_y;
	inicial_rddf_index = get_closest_rddf_index(&camera_pose_x, &camera_pose_y, camera_pose, board_pose, globalpos, last_rddf_poses);

	d.i = inicial_rddf_index;
	d.rddf_pos.x = last_rddf_poses.poses[inicial_rddf_index].x;
	d.rddf_pos.y = last_rddf_poses.poses[inicial_rddf_index].y;
	d.camera_pos.x = camera_pose_x;
	d.camera_pos.y = camera_pose_y;
	closest_rddf.push_back(d);

	double distance = 0.0, distance_ant = 0.0;
	double distance_accum = meters_spacement;

	for (int i = inicial_rddf_index; i < last_rddf_poses.number_of_poses; i++)
	{
		distance = euclidean_distance(camera_pose_x, last_rddf_poses.poses[i].x, camera_pose_y, last_rddf_poses.poses[i].y);
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


vector<bbox_t>
get_predictions_of_crops (int i, cv::Mat image, void *network_struct, char **classes_names)
{
	vector<bbox_t> predictions;
	stringstream ss;
	ss << i;
	string window_name = "slice_" + ss.str();
	cv::Mat src_image = image;
	cv::Mat rgb_image = image;
	unsigned char *img;
	img = image.data;
	predictions = run_YOLO(img, image.cols, image.rows, network_struct, classes_names, 0.5);//darknet->detect(src_image, 0.2);  // Arguments (img, threshold)
	return (predictions);
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


vector<bbox_t>
transform_bounding_boxes_of_crops (vector< vector<bbox_t> > bounding_boxes_of_crops, vector<t_transform_factor> transform_factor_of_slice_to_original_frame,
		cv::Mat or_image, char **classes_names)
{
	cv::Mat img;
	img = or_image.clone();
	vector<bbox_t> bboxes;
	bbox_t b;
	bool intersects_with_bboxes = false;
	bool rect_dont_intersects = false;
	bool intersect;
	for (int i = 0; i < bounding_boxes_of_crops.size(); i++)
	{
		for (int j = 0; j < bounding_boxes_of_crops[i].size(); j++)
		{
			intersect = false;

			int obj_id = bounding_boxes_of_crops[i][j].obj_id;
			string obj_name;
			obj_name = classes_names[obj_id];

			if (obj_name.compare("car") == 0)
			{
				b = bounding_boxes_of_crops[i][j];
				b.x = bounding_boxes_of_crops[i][j].x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
				b.y = bounding_boxes_of_crops[i][j].y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;

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


void
show_detections_alberto(vector<t_transform_factor> transform_factor_of_slice_to_original_frame, vector<cv::Mat> scene_crops,
		vector<vector<bbox_t>> bounding_boxes_of_crops, vector<bbox_t> predictions,
		vector<carmen_position_t> rddf_points_in_image_filtered, int qtd_crops, char **classes_names, char *groundtruth_path, double image_timestamp)
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
    // if (access(groundtruth_folder.c_str(), F_OK) == 0)
    // {
    // 	//cout<<groundtruth_folder<<" show"<<endl;
    // 	FILE *f;
    // 	f = fopen (groundtruth_folder.c_str(), "r");
    // 	//int yy = fscanf (f, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2);
    // 	//yy = yy;

    // 	while (fscanf (f, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2) != EOF)
    // 	{
    // 		gt.x = (int)x1;
    // 		gt.y = (int)y1;
    // 		gt.w = (int)(x2 - x1);
    // 		gt.h = (int)(y2 - y1);
    // 		cv::rectangle(scene_crops[0],
    // 				cv::Point(gt.x, gt.y),
	// 				cv::Point(gt.x + gt.w, gt.y + gt.h),
	// 				Scalar(0, 255, 0), 3);
    // 	}

    // }
    // else
    // 	exit(0);

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

    	for (int j = 0; j < bounding_boxes_of_crops[i].size(); j++)
    	{
    		//cout<<bounding_boxes_of_crops[i].size()<<endl;

    		bbox_t b = bounding_boxes_of_crops[i][j];
    		bbox_t b_print = bounding_boxes_of_crops[i][j];
    		b_print.x = b_print.x + transform_factor_of_slice_to_original_frame[i].translate_factor_x;
    		b_print.y = b_print.y + transform_factor_of_slice_to_original_frame[i].translate_factor_y;


    		cv::Scalar object_color;

    		//sprintf(confianca, "%d  %.3f", predictions.at(i).obj_id, predictions.at(i).prob);

    		int obj_id = bounding_boxes_of_crops[i][j].obj_id;

    		string obj_name;
    		obj_name = classes_names[obj_id];

    		//
			if (obj_name.compare("car") == 0)
			{
					object_color = cv::Scalar(0, 0, 255);
					line_tickness = 2;

					cv::rectangle(scene_crops[i],
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

    			cv::circle(scene_crops[0], cv::Point(rddf_points_in_image_filtered[l].x, rddf_points_in_image_filtered[l].y), 3.5, cv::Scalar(0, 255, 255), thickness, lineType);
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
//    				cv::rectangle(scene_crops[0],
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
    				cv::rectangle(scene_crops[0],
    						cv::Point(predictions[k].x, predictions[k].y),
							cv::Point(predictions[k].x + predictions[k].w, predictions[k].y + predictions[k].h),
							Scalar(0, 255, 255), 3);

    		}

    	}
    	cv:Mat aux_img = scene_crops[i];
    	if (i == 0)
    		cv::resize(aux_img, aux_img, Size(1152, 691));
    	else
    		cv::resize(aux_img, aux_img, Size(384, 230));
    	cv::imshow(name, aux_img);
    	cv::waitKey(1);
    }
    printf("******************************************\n\n\n\n\n\n");
}


vector<cv::Scalar>
get_slice_colors (unsigned int crops_size)
{
	vector<cv::Scalar> colors;
	cv::Scalar color;
	if (crops_size <= 1)
	{
		color = cv::Scalar (0, 0, 255);
		colors.push_back(color);
	}
	if (crops_size <= 2)
	{
		color = cv::Scalar (0, 255, 0);
		colors.push_back(color);
	}
	if (crops_size <= 3)
	{
		color = cv::Scalar (255, 0, 0);
		colors.push_back(color);
	}
	if (crops_size <= 4)
	{
		color = cv::Scalar (255, 255, 0);
		colors.push_back(color);
	}
	if (crops_size <= 5)
	{
		color = cv::Scalar (255, 0, 255);
		colors.push_back(color);
	}
	if (crops_size <= 6)
	{
		color = cv::Scalar (0, 255, 255);
		colors.push_back(color);
	}
	if (crops_size <= 7)
	{
		color = cv::Scalar (0, 0, 0);
		colors.push_back(color);
	}
	if (crops_size <= 8)
	{
		color = cv::Scalar (255, 255, 255);
		colors.push_back(color);
	}
	return (colors);
}


//bool before_first_file = true;
//bool acessessing = false;
//void
//save_detections(double timestamp, vector<bbox_t> bounding_boxes_of_slices_in_original_image, cv::Mat rgb_image,
//				vector<cv::Mat> scene_slices, vector<cv::Scalar> colors, vector<t_transform_factor> transform_factor_of_slice_to_original_frame,
//				vector<carmen_position_t> rddf_points_in_image_filtered, vector<carmen_position_t> rddf_points_in_image_full, vector<vector<bbox_t>> bounding_boxes_of_detections)
//{
//	vector<int> compression_params;
//	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//	compression_params.push_back(9);
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
//	string images_folder = str_folder_image_name_slices + arr + "-r";
//	string images_folder_det = str_folder_image_name_slices_and_detection + arr + "-r";
//	string images_folder_original = str_folder_image_name_original + arr + "-r.png";
//	string images_folder_rddf_filtered = str_folder_image_name_rddf_filtered + arr + "-r.png";
//	string images_folder_rddf_full = str_folder_image_name_rddf_full + arr + "-r.png";
//	string images_folder_slices_rddf_full = str_folder_image_name_slices_rddf_full + arr + "-r.png";
//	string images_folder_slices_rddf_filtered = str_folder_image_name_slices_rddf_filtered + arr + "-r.png";
//	cv::Mat rgb_image_original = rgb_image.clone();
//	cv::Mat rgb_image_all_slices = rgb_image.clone();
//
//
//	if (access(groundtruth_folder.c_str(), F_OK) == 0)
//	{
//
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
//			int retorno = fscanf (f_groundtruth, "%s %f %f %f %f", classe, &x1, &y1, &x2, &y2);
//			FILE *f_detection = fopen (detections_folder.c_str(), "w");
//			for (int i = 0; i < bounding_boxes_of_slices_in_original_image.size(); i++)
//			{
//				//cout<<"\t"<<i<<endl;
//				bbox_t b = bounding_boxes_of_slices_in_original_image[i];
//				int obj_id = b.obj_id;
//				//cout<<"\t"<<" "<<obj_names[obj_id]<<" "<<(float)b.x<<" "<<(float)b.y<<" "<<(float)(b.x + b.w)<<" "<<(float)(b.y + b.h)<<endl;
//				string obj_name;
//				if (obj_names.size() > obj_id)
//					obj_name = obj_names[obj_id];
//
//				if (obj_name.compare("car") == 0)
//				{
//					cv::Rect rect_A;
//					cv::Rect rect_B;
//					rect_A.x = (int)x1; rect_A.y = (int)y1;
//					rect_A.width = (int)x2 - (int)x1; rect_A.height = (int)y2 - (int)y1;
//					rect_B.x = (int)b.x; rect_B.y = (int)b.y;
//					rect_B.width = (int)b.w; rect_B.height = b.h;
//										cv::Point l1, r1, l2, r2;
//										l1.x = (int)x1; l1.y = (int)y1; //top left
//										r1.x = (int)x2; r1.y = (int)y2; //right botton of groundtruth bbox
//										l2.x = (int)b.x; l2.y = (int)b.y; //top left
//										r2.x = (int)b.x + b.w; r2.y = (int)b.y + b.h; //right botton of detection
//					//cout<<x1<<" "<<x1<<" "
//					//cout<<classe<<" "<<x1<<" "<<y1<<" "<<x2<<" "<<y2<<endl;
//					//if(rectangles_intersects(l1, r1, l2, r2))
////										calc_percentage_of_rectangles_intersection(l1, r1, l2, r2) > 50
////					if(rectangles_intersects(rect_A, rect_B))
////					{
//						//cout<<"\t"<<i<<" "<<obj_names[obj_id]<<endl;
//						fprintf (f_detection, "%s %f %.2f %.2f %.2f %.2f\n", "car", b.prob, (float)b.x, (float)b.y, (float)(b.x + b.w), (float)(b.y + b.h));
////					}
//				}
//
//			}
//			fclose (f_detection);
//			fclose (f_groundtruth);
//		}
//
//	}
//	else
//		acessessing = false;
//
//	if (before_first_file == false && acessessing == false)
//	{
//		cout<<"database_completed!"<<endl;
//		exit(0);
//	}
//}

#include "v_disparity.h"

#ifndef NO_CUDA
#include "v_disparity_GPU.h"
#endif

#include <vector>

v_disparity get_v_disparity_instance(stereo_util stereo_util_instance, int stereo_disparity)
{
	v_disparity instance;

	instance.stereo_util_instance = stereo_util_instance;
	instance.stereo_disparity = stereo_disparity;
	instance.memory_storage = cvCreateMemStorage(0);
	instance.polygon_storage = cvCreateMemStorage(0);
	instance.polygon = cvCreateSeq(CV_SEQ_CONTOUR, sizeof(CvSeq), sizeof(CvPoint), instance.polygon_storage);

	int n_edges = 4;
	//CvPoint edges[4] = {cvPoint(80, 380), cvPoint(200, 220), cvPoint(280, 220), cvPoint(400, 380)};
	//This are the points that make the blue region (safe region) in the v-disparity viewer
	CvPoint edges[4] = {cvPoint(170, 357), cvPoint(284, 270), cvPoint(340, 270), cvPoint(412, 357)}; // <- calibrated for camera 8 at 640 x 480 resolution

	build_polygon(edges, n_edges, instance);

#ifndef NO_CUDA

	init_v_disparity_GPU(stereo_util_instance.width, stereo_util_instance.height, 3);

#endif

	return instance;
}

void destroy_v_disparity_instance(v_disparity instance)
{
	if (instance.memory_storage != NULL)
		cvReleaseMemStorage(&instance.memory_storage);
	if (instance.polygon_storage != NULL)
		cvReleaseMemStorage(&instance.polygon_storage);
}

void build_polygon(CvPoint *edges, int n_edges, v_disparity instance)
{
	for (int i = 0; i < n_edges; i++)
	{
		CvPoint edge = edges[i];
		cvSeqPush(instance.polygon, &edge);
	}
}

int get_v_disparity_size(v_disparity instance)
{
	int stereo_height = instance.stereo_util_instance.height;
	int stereo_disparity = instance.stereo_disparity;

	return stereo_height * stereo_disparity;
}

int get_u_disparity_size(v_disparity instance)
{
	int stereo_width = instance.stereo_util_instance.width;
	int stereo_disparity = instance.stereo_disparity;

	return stereo_width * stereo_disparity;
}

static double
get_line_slope(CvPoint A, CvPoint B, int image_height)
{
	int A_y = image_height - A.y - 1;
	int B_y = image_height - B.y - 1;
	int dX = abs(A.x - B.x);
	int dY = abs(A_y - B_y);

	return atan2(dY, dX);
}

double get_slope_normalized_in_degrees(CvPoint A, CvPoint B, int image_height)
{
	double theta = get_line_slope(A, B, image_height);
	return 180 - carmen_radians_to_degrees(theta);
}

void resize_line(double resize_factor, CvPoint *A, CvPoint *B, v_disparity instance)
{
	/* Resize the segments a little */
	double lenght = sqrt(pow(A->x - B->x, 2) + pow(A->x - B->y, 2));
	double theta = get_line_slope(*A, *B, instance.stereo_util_instance.height);
	double dX = cos(theta);
	double dY = sin(theta);

	if (A->x < B->x)
	{
		A->x = cvRound(A->x - resize_factor * lenght * dX);
		B->x = cvRound(B->x + resize_factor * lenght * dX);
	}
	else
	{
		A->x = cvRound(A->x + resize_factor * lenght * dX);
		B->x = cvRound(B->x - resize_factor * lenght * dX);
	}

	if (A->y > B->y)
	{
		A->y = cvRound(A->y + resize_factor * lenght * dY);
		B->y = cvRound(A->y - resize_factor * lenght * dY);
	}
	else
	{
		A->y = cvRound(A->y - resize_factor * lenght * dY);
		B->y = cvRound(B->y + resize_factor * lenght * dY);
	}
}



cvLineSegment *find_hough_lines(const IplImage *image, int *n_lines, v_disparity instance)
{
	IplImage *ch1_image = cvCloneImage(image);

	/* Morphological skeleton of the image */
#if CV_MAJOR_VERSION == 2
	cv::Mat img(ch1_image);
#elif CV_MAJOR_VERSION == 3
	cv::Mat img = cv::cvarrToMat(ch1_image);
#endif
	cv::threshold(img, img, 127, 255, cv::THRESH_BINARY);
	cv::Mat skel(img.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp(img.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat eroded(img.size(), CV_8UC1, cv::Scalar(0));

	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	bool done;
	do
	{
	  cv::erode(img, eroded, element);
	  cv::dilate(eroded, temp, element); // temp = open(img)
	  cv::subtract(img, temp, temp);
	  cv::bitwise_or(skel, temp, skel);
	  eroded.copyTo(img);

	  done = (cv::countNonZero(img) == 0);
	} while (!done);
	//skel is the result of this process, the other Mat's so far were only to help
	//cv::imshow("Skeleton", skel);


	/*  Apply Hough transform to find lines */
	std::vector<cv::Vec4i> lines;

	cv::HoughLinesP(skel, lines, 1, CV_PI/360.0, 60, 45, 10);

	cvLineSegment *found_lines = new cvLineSegment[lines.size()];
	*n_lines = lines.size();

	for( size_t i = 0; i < lines.size(); i++ )
	{

		found_lines[i].A = cvPoint(lines[i][0],lines[i][1]);
		found_lines[i].B = cvPoint(lines[i][2],lines[i][3]);
	}

	cvClearMemStorage(instance.memory_storage);

	return found_lines;
}

void compute_height_and_pitch(IplImage *v_disparity_map, double slope_threshould, cvLineSegment *lines, int n_lines, double *height, double *pitch, double *horizon_line, v_disparity instance)
{
	static double angular_coef, linear_coef;
	//double angular_coef = 0.0, linear_coef = 0.0;
	int best_sum = 0;

	for (int i = 0; i < n_lines; i++)
	{
		CvPoint A = cvPoint(lines[i].A.x, lines[i].A.y);
		CvPoint B = cvPoint(lines[i].B.x, lines[i].B.y);

		resize_line(2.0, &A, &B, instance);

		double angle = get_slope_normalized_in_degrees(A, B, instance.stereo_util_instance.height);
		if (angle < slope_threshould)
			continue;

		CvLineIterator line_iterator;
		int n_points = cvInitLineIterator(v_disparity_map, A, B, &line_iterator, 8, 0);
		int sum = 0.0;

		for(int i = 0; i < n_points; i++)
		{
			sum += line_iterator.ptr[0];
			CV_NEXT_LINE_POINT(line_iterator);
		}

		if (sum > best_sum)
		{
			best_sum = sum;

			double theta = atan2(B.y - A.y, B.x - A.x);
			linear_coef = A.y - tan(theta) * A.x;
			angular_coef = tan(0.5 * CV_PI - theta);

		}
	}

	*pitch = atan((instance.stereo_util_instance.yc - linear_coef) / instance.stereo_util_instance.fx);
	*height = instance.stereo_util_instance.baseline * cos(*pitch) / angular_coef;
	*horizon_line = linear_coef;

	/*Debug Code*/
	if(*height > 100)
	{
		printf("Infinite Bug Alert!\n");
		printf("Height: %f\n", *height);
		printf("best sum: %d, angular coef: %f, baseline: %f, cos(pitch): %f;\n",best_sum, angular_coef, instance.stereo_util_instance.baseline, cos(*pitch));
		printf("yc: %f, linear coef: %f, fx: %f\n",instance.stereo_util_instance.yc , linear_coef, instance.stereo_util_instance.fx);
	}

	if(*height < 1){
		printf("low height Alert!\n");
		printf("Height: %f\n", *height);
		printf("best sum: %d, angular coef: %f, baseline: %f, cos(pitch): %f;\n",best_sum, angular_coef, instance.stereo_util_instance.baseline, cos(*pitch));
		printf("yc: %f, linear coef: %f, fx: %f\n",instance.stereo_util_instance.yc , linear_coef, instance.stereo_util_instance.fx);
	}

	// End of Debug Code


}


unsigned short int *alloc_v_disparity_map(v_disparity instance)
{
	int stereo_height = instance.stereo_util_instance.height;
	int stereo_disparity = instance.stereo_disparity;
	int v_disparity_size = stereo_height * stereo_disparity;

	unsigned short int *v_disparity_data = (unsigned short int*)malloc(v_disparity_size * sizeof(unsigned short int));
	carmen_test_alloc(v_disparity_data);

	return v_disparity_data;
}

void fill_v_disparity_map(unsigned short int *v_disparity_data, float *disparity_map, v_disparity instance)
{
	int x, y;
	int stereo_width = instance.stereo_util_instance.width;
	int stereo_height = instance.stereo_util_instance.height;
	int stereo_disparity = instance.stereo_disparity;
	int v_disparity_size = stereo_height * stereo_disparity;

	memset(v_disparity_data, 0, v_disparity_size * sizeof (unsigned short int)); // set all disparity accumulators to zero

	unsigned short int *v_disparity_offset;
	for(y = 0;y < stereo_height; y++)
	{
		v_disparity_offset = &(v_disparity_data[y * stereo_disparity]);
		for(x = 0;x < stereo_width; x++)
		{
			int disparity_value = (int)(disparity_map[y * stereo_width + x]);
			disparity_value = MAX(0, disparity_value); // ensure no negative disparity
			disparity_value = MIN(disparity_value, stereo_disparity - 1); //ensure no disparity above max
			v_disparity_offset[disparity_value]++;// add one to the disparity value accumulator}
		}
	}
}

unsigned short int *alloc_u_disparity_map(v_disparity instance)
{
	int stereo_width = instance.stereo_util_instance.width;
	int stereo_disparity = instance.stereo_disparity;
	int u_disparity_size = stereo_width * stereo_disparity;

	unsigned short int *u_disparity_data = (unsigned short int*)malloc(u_disparity_size * sizeof(unsigned short int));
	carmen_test_alloc(u_disparity_data);

	return u_disparity_data;
}

void fill_u_disparity_map(unsigned short int *u_disparity_data, float *disparity_map, v_disparity instance)
{
	int x, y;
	int stereo_width = instance.stereo_util_instance.width;
	int stereo_height = instance.stereo_util_instance.height;
	int stereo_disparity = instance.stereo_disparity;
	int u_disparity_size = stereo_width * stereo_disparity;

	memset(u_disparity_data, 0, u_disparity_size * sizeof (unsigned short int)); // set all disparity accumulators to zero

	for (y = 0; y < stereo_height; y++)
	{
		for(x = 0; x < stereo_width; x++)
		{
			int disparity_value = (int)(disparity_map[y * stereo_width + x]);
			u_disparity_data[x * stereo_disparity + disparity_value]++;// add one to the disparity value accumulator
		}
	}
}

IplImage *alloc_u_disparity_map_image(v_disparity instance)
{
	int stereo_width = instance.stereo_util_instance.width;
	int stereo_disparity = instance.stereo_disparity;

	IplImage *u_disparity_map = cvCreateImage(cvSize(stereo_disparity, stereo_width), IPL_DEPTH_8U, 1);

	return u_disparity_map;
}

void fill_u_disparity_image(unsigned short int *u_disparity_data, IplImage *u_disparity_map, v_disparity instance)
{
	int u_disparity_size = get_u_disparity_size(instance);

	for (int i = 0; i < u_disparity_size; i++)
	{
		u_disparity_map->imageData[i] = u_disparity_data[i] > 255 ? 255 : u_disparity_data[i];
	}
}

IplImage *alloc_v_disparity_map_image(v_disparity instance)
{
	int stereo_height = instance.stereo_util_instance.height;
	int stereo_disparity = instance.stereo_disparity;

	IplImage *v_disparity_map = cvCreateImage(cvSize(stereo_disparity, stereo_height), IPL_DEPTH_8U, 3);

	return v_disparity_map;
}

void fill_v_disparity_image(unsigned short int *v_disparity_data, IplImage *v_disparity_map, v_disparity instance)
{
	int x, y, xi;
	int stereo_height = instance.stereo_util_instance.height;
	int stereo_disparity = instance.stereo_disparity;
	double inverse_scale_factor = (double) stereo_disparity / (double) v_disparity_map->width;
	unsigned short int value;

	for (y = 0; y < stereo_height; y++)
	{
		for (x = 0; x < v_disparity_map->width; x++)
		{
			xi = (inverse_scale_factor < 1.0) ? (x * inverse_scale_factor + inverse_scale_factor / 2.0) : x;
			value = v_disparity_data[y * stereo_disparity + xi] > 255 ? 255 : v_disparity_data[y * stereo_disparity + xi];
			v_disparity_map->imageData[y * v_disparity_map->width + x] = (unsigned char)value;
		}
	}
}

int draw_road_profile_lines_in_v_disparity(IplImage *road_profile_image, IplImage *v_disparity_map, double slope_threshould, v_disparity instance)
{
	cvSet(road_profile_image, cvScalar(0,0,0,0), NULL);//clear the road_profile_image

	int n_lines;
	cvLineSegment *lines = find_hough_lines(v_disparity_map, &n_lines, instance);

	cvLineSegment road_profile_line;
	road_profile_line.A = cvPoint(0, 0);
	road_profile_line.B = cvPoint(0, 0);
	int n_road_lines = 0;
	int best_sum = 0;

	for (int i = 0; i < n_lines; i++)
	{
		CvPoint A = lines[i].A;
		CvPoint B = lines[i].B;

		double angle = get_slope_normalized_in_degrees(A, B, v_disparity_map->height);

		if (angle > slope_threshould)
		{
			n_road_lines++;

			CvLineIterator line_iterator;
			int n_points = cvInitLineIterator(v_disparity_map, A, B, &line_iterator, 8, 0);
			int sum = 0.0;

			for(int i = 0; i < n_points; i++)
			{
				sum += line_iterator.ptr[0];
				CV_NEXT_LINE_POINT(line_iterator);
			}

			if (sum > best_sum)
			{
				best_sum = sum;
				road_profile_line.A = A;
				road_profile_line.B = B;
			}
		}
	}

	resize_line(1.2, &road_profile_line.A, &road_profile_line.B, instance);
	#ifdef UBUNTU_20_04
		cvLine(road_profile_image, road_profile_line.A, road_profile_line.B, cvScalar(0,255,0), 3, 4, 0);
	#else
		cvLine(road_profile_image, road_profile_line.A, road_profile_line.B, CV_RGB(0,255,0), 3, 4, 0);
	#endif

	return n_road_lines;
}


void compute_v_disparity_info(unsigned short int *v_disparity_data, IplImage *v_disparity_map, double slope_threshould, float *disparity_map, double *height, double *pitch, double *horizon_line, v_disparity instance)
{
	// fill the v-disparity buffer and the v-disparity mao image with info
	fill_v_disparity_map(v_disparity_data, disparity_map, instance);
	fill_v_disparity_image(v_disparity_data, v_disparity_map, instance);

	// find the lines on v-disparity image
	int n_lines = 0;
	cvLineSegment *lines = find_hough_lines(v_disparity_map, &n_lines, instance);

	// compute the camera height, pitch and horizon line
	compute_height_and_pitch(v_disparity_map, slope_threshould, lines, n_lines, height, pitch, horizon_line, instance);

	// Clean up memory
	delete(lines);
}

static int
is_possible_obstacle(IplImage *road_profile_image, int y, int disparity, v_disparity instance)
{
	double scale_factor = (double) road_profile_image->width / (double) instance.stereo_disparity;

	CvScalar possible_obstacle;
	if (scale_factor > 1.0)
		possible_obstacle = cvGet2D(road_profile_image, y, disparity * scale_factor + scale_factor / 2.0);
	else
		possible_obstacle = cvGet2D(road_profile_image, y, disparity);

	// Discard all pixels that are outside the road profile, i.e, that are colored pixels in the road profile
	if ((possible_obstacle.val[0] == 0) && (possible_obstacle.val[1] == 0) && (possible_obstacle.val[2] == 0))
		return (1);
	else
		return (0);
}


static int
v_disparity_obstacle(IplImage *v_disparity_map, int y, int disparity, v_disparity instance)
{
	double scale_factor = (double) v_disparity_map->width / (double) instance.stereo_disparity;
	CvScalar v_disparity_value;

	if (scale_factor > 1.0)
		v_disparity_value = cvGet2D(v_disparity_map, y, disparity * scale_factor + scale_factor / 2.0);
	else
		v_disparity_value = cvGet2D(v_disparity_map, y, disparity);

	// consider as a possible obstacle if there are more than 16 pixels with the disparity in the same line
	// we can verify the pixel size in meters and use this threshould in meters
	if (v_disparity_value.val[0] > 32)// one needs to test only one channel because the image is grayscale
		return (1);
	else
		return (0);
}

static int
u_disparity_obstacle(unsigned short int *u_disparity_map, int y, int disparity, v_disparity instance)
{
	int stereo_disparity = instance.stereo_disparity;

	unsigned short u_disparity_value = u_disparity_map[y * stereo_disparity + disparity];

	if (u_disparity_value > 12)
		return (1);
	else
		return (0);
}

static void
fill_pixel_color_for_obstacle(CvScalar *pixel, carmen_vector_3D_p p_world, double camera_height)
{
	double min_obstacle_height = MINIMUM_OBSTACLE_HEIGHT(camera_height);
	double max_obstacle_height = MAXIMUM_OBSTACLE_HEIGHT(camera_height);

	double distance = sqrt(p_world->x * p_world->x + p_world->y * p_world->y + p_world->z * p_world->z);
	if ((distance < RED_OBSTACLES_DISTANCE) && //all obstacles that are less than 10 meters are red
			(p_world->z > min_obstacle_height) && (p_world->z < max_obstacle_height))
	{
		pixel->val[0] = 255; // red obstacles
	}
	else if ((distance < GREEN_OBSTACLES_DISTANCE) && //all obstacles that are between 10 and 20 meters are green
			(p_world->z > min_obstacle_height) && (p_world->z < max_obstacle_height))
	{
		pixel->val[0] = pixel->val[1] = 255; // yellow obstacles
	}
	else
	{
		pixel->val[1] = 255; // green obstacles
	}
}

void print_obstacles_OpenMp(IplImage *dst_image, IplImage *v_disparity_map, IplImage *road_profile_image,
		float *disparity_map, double camera_height, double camera_pitch,
		v_disparity instance)
{
	int stereo_width = instance.stereo_util_instance.width;
	int stereo_disparity = instance.stereo_disparity;

	unsigned short int *u_disparity_map = alloc_u_disparity_map(instance);
	fill_u_disparity_map(u_disparity_map, disparity_map, instance);

#ifndef DEBUG
#pragma omp parallel for
#endif
	for (int disparity = 1; disparity < stereo_disparity-1; disparity++)
	{
		for (int y = 0; y < v_disparity_map->height; y++)
		{
			if (!is_possible_obstacle(road_profile_image, y, disparity, instance))
			{
				continue;
			}else if (v_disparity_obstacle(v_disparity_map, y, disparity, instance))
			{
				for (int x = 0; x < dst_image->width; x++)
				{
					if (!u_disparity_obstacle(u_disparity_map, x, disparity, instance))
					{
						continue;
					}

					int disparity_value = (int)disparity_map[y * stereo_width + x];
					if (disparity_value == disparity)
					{
						CvScalar pixel_value = cvGet2D(dst_image, y, x);

						carmen_position_t right_point;
						right_point.x = x;
						right_point.y = y;
						carmen_vector_3D_p P_raw = reproject_single_point_to_3D(&instance.stereo_util_instance, right_point, disparity);
						if (P_raw == NULL)
							continue;

						carmen_vector_3D_p P_world = transform_camera_to_world_frame(P_raw, camera_pitch);
						if (P_world == NULL)
						{
							free(P_raw);
							continue;
						}

						fill_pixel_color_for_obstacle(&pixel_value, P_world, camera_height);
						//cvSet2D(dst_image, y, x, pixel_value);

						free(P_raw);
						free(P_world);
					}
				}
			}
		}
	}

	for (int y = 0; y < v_disparity_map->height; y++)
	{
		for (int x = 0; x < dst_image->width; x++)
		{
			if (cvPointPolygonTest(instance.polygon, cvPoint2D32f(x, y), 0) >= 0)
			{
				CvScalar pixel_value = cvGet2D(dst_image, y, x);
				pixel_value.val[2] = 255; // blue area
				cvSet2D(dst_image, y, x, pixel_value);
			}
		}
	}

	free(u_disparity_map);
}


void print_obstacles(IplImage *dst_image, IplImage *v_disparity_map, IplImage *road_profile_image,
		float *disparity_map, double camera_height, double camera_pitch,
		v_disparity instance)
{


#ifndef NO_CUDA
	print_obstacles_GPU(
			(unsigned char *)dst_image->imageData,
			(unsigned char *)v_disparity_map->imageData,
			(unsigned char *)road_profile_image->imageData,
			disparity_map,
			camera_height,
			camera_pitch,
			instance.stereo_util_instance.baseline,
			instance.stereo_util_instance.fx,
			instance.stereo_util_instance.xc,
			instance.stereo_util_instance.yc,
			instance.stereo_util_instance.width,
			instance.stereo_util_instance.height,
			instance.stereo_disparity);
#else

	print_obstacles_OpenMp(dst_image, v_disparity_map, road_profile_image,
			disparity_map, camera_height,camera_pitch,
			instance);
#endif
}

void print_u_disparity_obstacles(IplImage *dst_image, IplImage *u_disparity_map, float *disparity_map, double camera_height, double camera_pitch,
		v_disparity instance)
{
	int stereo_width = instance.stereo_util_instance.width;
	int n_lines;
	cvLineSegment *lines = find_hough_lines(u_disparity_map, &n_lines, instance);

	for (int i = 0; i < n_lines; i++)
	{
		CvPoint A = lines[i].A;
		CvPoint B = lines[i].B;

		if (abs(A.x - B.x) <= 1)
		{
			for (int x = MIN(A.y, B.y); x < MAX(A.y, B.y); x++)
			{
				for (int y = 0; y < 420; y++)
				{
					int disparity_value = (int)disparity_map[y * stereo_width + x];
					if (disparity_value == A.x || disparity_value == B.x)
					{
						CvScalar pixel_value = cvGet2D(dst_image, y, x);

						carmen_position_t right_point;
						right_point.x = x;
						right_point.y = y;
						carmen_vector_3D_p P_raw = reproject_single_point_to_3D(&instance.stereo_util_instance, right_point, disparity_value);
						carmen_vector_3D_p P_world = transform_camera_to_world_frame(P_raw, camera_pitch);

						fill_pixel_color_for_obstacle(&pixel_value, P_world, camera_height);
						cvSet2D(dst_image, y, x, pixel_value);

						free(P_raw);
						free(P_world);
					}
				}
			}
		}
	}

}


int get_road_pixels_list(IplImage *src_image, CvPoint *points, CvScalar *samples,
		IplImage *road_profile_image, float *disparity_map, int only_inside_polygon, v_disparity instance)
{
	int stereo_width = instance.stereo_util_instance.width;
	int stereo_height = instance.stereo_util_instance.height;
	int stereo_disparity = instance.stereo_disparity;
	int n_points = 0;
	int x_i = 0, y_i = 0, x_f = stereo_width, y_f = stereo_height;

	if (only_inside_polygon)
	{
		x_i = INT_MAX;
		y_i = INT_MAX;
		x_f = INT_MIN;
		y_f = INT_MIN;
		for (int i = 0; i < 4; i++)
		{
			CvPoint *P = (CvPoint*)cvGetSeqElem(instance.polygon, i);
			x_i = MIN(x_i, P->x);
			y_i = MIN(y_i, P->y);
			x_f = MAX(x_f, P->x);
			y_f = MAX(y_f, P->y);
		}
	}

	for (int disparity = 1; disparity < stereo_disparity; disparity++)
	{
		for (int y = y_i; y < y_f; y++)
		{
			if (!is_possible_obstacle(road_profile_image, y, disparity, instance))
			{
				for (int x = x_i; x < x_f; x++)
				{
					int disparity_value = (int)disparity_map[y * stereo_width + x];
					if (disparity_value == disparity)
					{
						if (!only_inside_polygon || cvPointPolygonTest(instance.polygon, cvPoint2D32f(x, y), 0) >= 0)
						{
							points[n_points] = cvPoint(x, y);
							samples[n_points] = cvGet2D(src_image, y, x);
							n_points++;
						}
					}
				}
			}
		}
	}

	return n_points;
}


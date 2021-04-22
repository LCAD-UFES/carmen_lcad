#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <carmen/carmen.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

#define rrt_obstacle_probability_threshold 0.5			 // carmen-ford-escape.ini
#define IS_OBSTACLE rrt_obstacle_probability_threshold   // Minimum probability of a map cell being an obstacle

int true_positives = 0;
int true_negatives = 0;
int false_positives = 0;
int false_negatives = 0;
int true_positives_N = 0;
int true_negatives_N = 0;
int false_positives_N = 0;
int false_negatives_N = 0;

char *save_png_path = NULL;
char *poses_filename = NULL;
char *save_complete_map_path = NULL;
vector<carmen_point_t> poses_array;
carmen_point_t from_pose = {0.0, 0.0, 0.0};
carmen_point_t to_pose = {0.0, 0.0, 0.0};
double radius = 0.0;
bool show_map_changes = false;
bool show_complete_map = false;

Mat full_map;


cv::Mat
get_area_of_interest_from_poses_array(carmen_map_config_t map_config, double radius)
{
	if (poses_array.empty() || radius <= 0.0)
		return cv::Mat(cv::Size(map_config.x_size, map_config.y_size), CV_8UC3, cv::Scalar(1,1,1));

	cv::Mat area_of_interest = cv::Mat(cv::Size(map_config.x_size, map_config.y_size), CV_8UC3, cv::Scalar(2,2,2));
	int area_radius = (double) radius / map_config.resolution;

	for (int i = 0; i < (int) poses_array.size(); i++)
	{
		int x = (double) (poses_array[i].x - map_config.x_origin) / map_config.resolution;
		int y = (double) (poses_array[i].y - map_config.y_origin) / map_config.resolution;

		if (x >= -area_radius && x < (map_config.x_size + area_radius) && y >= -area_radius && y < (map_config.y_size + area_radius))
			circle(area_of_interest, cv::Point(x, map_config.y_size - 1 - y), area_radius, cv::Scalar(1,1,1), -1, 8, 0);
	}

	return area_of_interest;
}


cv::Vec3b
transform_map_to_image(double gt_cell, double clean_cell, double dirty_cell, bool update_stats)
{
	cv::Vec3b cell_color = cv::Vec3b(255, 144, 30); // bluish
	int update = int(update_stats);

	if (gt_cell < 0.0 || clean_cell < 0.0 || dirty_cell < 0.0)
		return cell_color;

	if (gt_cell <= IS_OBSTACLE && clean_cell <= IS_OBSTACLE  && dirty_cell <= IS_OBSTACLE)
	{
		true_negatives += update;
		cell_color = cv::Vec3b(255, 255, 255); // white
	}
	else if (gt_cell <= IS_OBSTACLE && clean_cell <= IS_OBSTACLE  && dirty_cell >  IS_OBSTACLE)
	{
		true_positives += update;
		cell_color = cv::Vec3b(  0, 175,   0); // green
	}
	else if (gt_cell <= IS_OBSTACLE && clean_cell >  IS_OBSTACLE  && dirty_cell <= IS_OBSTACLE)
	{
		false_positives_N += update;
		cell_color = cv::Vec3b(  0, 127, 127); // brown
	}
	else if (gt_cell <= IS_OBSTACLE && clean_cell >  IS_OBSTACLE  && dirty_cell >  IS_OBSTACLE)
	{
		false_negatives_N += update;
		cell_color = cv::Vec3b(  0,   0, 255); // red
	}
	else if (gt_cell >  IS_OBSTACLE && clean_cell <= IS_OBSTACLE  && dirty_cell <= IS_OBSTACLE)
	{
		false_negatives += update;
		cell_color = cv::Vec3b(  0,   0, 127); // dark red
	}
	else if (gt_cell >  IS_OBSTACLE && clean_cell <= IS_OBSTACLE  && dirty_cell >  IS_OBSTACLE)
	{
		false_positives += update;
		cell_color = cv::Vec3b( 31, 172, 255); // orange
	}
	else if (gt_cell >  IS_OBSTACLE && clean_cell >  IS_OBSTACLE  && dirty_cell <= IS_OBSTACLE)
	{
		true_positives_N += update;
		cell_color = cv::Vec3b(  0, 100,   0); // dark green
	}
	else if (gt_cell >  IS_OBSTACLE && clean_cell >  IS_OBSTACLE  && dirty_cell >  IS_OBSTACLE)
	{
		true_negatives_N += update;
		cell_color = cv::Vec3b(  0,   0,   0); // black
	}

	return cell_color;
}

void
compute_complete_map(Mat img, double map_resolution, int x_origin, int y_origin)
{
	static int min_x = INT_MAX, min_y = INT_MAX, max_x = 0, max_y = 0;

	if(full_map.empty())
	{
		full_map = img;
		min_x = x_origin;
		min_y = y_origin;
		max_x = x_origin;
		max_y = y_origin;
	}
	else
	{
		// int px = ((double)(x_origin - min_x)) / map_resolution;
		// int py = full_map.rows - 1 - ((double)(y_origin - min_y) / map_resolution);

		if ((int)x_origin < min_x)
		{
			// printf("Size1 %d %d Black %dx%d\n", (int)x_origin, min_x, (int)((min_x - (int) x_origin) / map_resolution), full_map.rows);
			Mat all_black_img = Mat::zeros(Size((int)((min_x - (int) x_origin) / map_resolution), full_map.rows), CV_8UC3);

			hconcat(all_black_img, full_map, full_map);
			// printf("Black %dx%d Full %dx%d\n", all_black_img.cols, all_black_img.rows, full_map.cols, full_map.rows);

			min_x = (int) x_origin;
		}
		if ((int) x_origin > max_x)
		{
			// printf("Size2  %d %d Black %dx%d\n", (int) x_origin, max_x, (int)(((int) x_origin - max_x) / map_resolution), full_map.rows);
			Mat all_black_img = Mat::zeros(Size(((int) x_origin - max_x) / map_resolution, full_map.rows), CV_8UC3);

			hconcat(full_map, all_black_img, full_map);
			// printf("Black %dx%d Full %dx%d\n", all_black_img.cols, all_black_img.rows, full_map.cols, full_map.rows);
		
			max_x = (int) x_origin;
		}
		if ((int) y_origin < min_y)
		{
			// printf("Size3  %d %d Black %dx%d\n", (int) y_origin, min_y, full_map.cols, (int)((min_y - (int) y_origin) / map_resolution));
			Mat all_black_img = Mat::zeros(Size(full_map.cols, ((min_y - (int) y_origin) / map_resolution)), CV_8UC3);
			
			vconcat(full_map, all_black_img, full_map);
			// printf("Black %dx%d Full %dx%d\n", all_black_img.cols, all_black_img.rows, full_map.cols, full_map.rows);
			
			min_y = (int) y_origin;
		}
		if ((int) y_origin > max_y)
		{
			// printf("Size4 %d %d Black %dx%d\n", (int) y_origin, max_y, full_map.cols, (int)(((int) y_origin - max_y) / map_resolution));
			Mat all_black_img = Mat::zeros(Size(full_map.cols, (((int) y_origin - max_y) / map_resolution)), CV_8UC3);

			vconcat(all_black_img, full_map, full_map);
			// printf("Black %dx%d Full %dx%d\n", all_black_img.cols, all_black_img.rows, full_map.cols, full_map.rows);
			
			max_y = (int) y_origin;
		}

		int px = ((double)(x_origin - min_x)) / map_resolution;
		int py = full_map.rows - 350 - ((double)(y_origin - min_y) / map_resolution);

		// printf("---------------- FM YM %d GT YM %d\n", min_y, (int) y_origin);
		// printf("---------------- FM %dx%d GT %dx%d\n", full_map.cols, full_map.rows, px, py);

		img.copyTo(full_map(Rect(px, py, 350, 350)));
	}

	if (show_complete_map)
	{
		Mat copy_full_map = full_map.clone();
		resize(copy_full_map, copy_full_map, Size(copy_full_map.cols * 0.05, copy_full_map.rows * 0.05), INTER_NEAREST);
		imshow("Full Map", copy_full_map); waitKey(1);
	}

	
	// imwrite(save_complete_map_path, full_map);
	// imwrite("Full.png", full_map);
}

void
compute_one_map_changes(char *gt_path, char *clean_path, char *dirty_path, char *save_path)
{
	carmen_map_t gt_map, clean_map, dirty_map;

	if (carmen_map_read_gridmap_chunk(gt_path, &gt_map) != 0)
	{
		fprintf(stderr, "Could not read gt_map: %s\n", gt_path);
		return;
	}
	// printf("Opening gt_map: %s\n", gt_path);

	if (carmen_map_read_gridmap_chunk(clean_path, &clean_map) != 0)
	{
		fprintf(stderr, "Could not read map: %s\n", clean_path);
		carmen_map_free_gridmap(&gt_map);
		return;
	}
	// printf("Opening clean_map: %s\n", clean_path);

	if (carmen_map_read_gridmap_chunk(dirty_path, &dirty_map) != 0)
	{
		fprintf(stderr, "Could not read map: %s\n", dirty_path);
		carmen_map_free_gridmap(&gt_map);
		carmen_map_free_gridmap(&clean_map);
		return;
	}
	// printf("Opening dirty_map: %s\n", dirty_path);

	char *filename = strrchr(gt_path, '/') + 1;
	sscanf(filename, "m%lf_%lf.map", &gt_map.config.x_origin, &gt_map.config.y_origin);

	cv::Mat img = cv::Mat(cv::Size(gt_map.config.x_size, gt_map.config.y_size), CV_8UC3);
	cv::Mat shade = get_area_of_interest_from_poses_array(gt_map.config, radius);

	for (int x = 0; x < gt_map.config.x_size; x++)
	{
		for (int y = 0; y < gt_map.config.y_size; y++)
		{
			cv::Point map_cell = cv::Point(x, gt_map.config.y_size - 1 - y);
			bool update_stats = (shade.at<cv::Vec3b>(map_cell) == cv::Vec3b(1,1,1));
			cv::Vec3b cell_color = transform_map_to_image(gt_map.map[x][y], clean_map.map[x][y], dirty_map.map[x][y], update_stats);
			img.at<cv::Vec3b>(map_cell) = cell_color;
		}
	}

	carmen_map_free_gridmap(&gt_map);
	carmen_map_free_gridmap(&clean_map);
	carmen_map_free_gridmap(&dirty_map);

	img = img / shade;

	if (save_complete_map_path)
		compute_complete_map(img, gt_map.config.resolution, gt_map.config.x_origin, gt_map.config.y_origin);

	resize(img, img, Size(0, 0), 2.5, 2.5, INTER_NEAREST);

	if (save_path)
		imwrite(save_path, img);

	if (show_map_changes)
	{
		namedWindow(filename);
		moveWindow(filename, 10, 10);
		imshow(filename, img);
		waitKey(0);
		destroyWindow(filename);
	}
}


void
compute_metrics()
{
	double precision = 0.0;
	double recall    = 0.0;
	double accuracy  = 0.0;

	printf("\nTrue Positives    (%8d) green\n", true_positives);
	printf("True Positives_N  (%8d) dark green\n", true_positives_N);
	printf("True Negatives    (%8d) white\n", true_negatives);
	printf("True Negatives_N  (%8d) black\n", true_negatives_N);
	printf("False Positives   (%8d) orange\n", false_positives);
	printf("False Positives_N (%8d) brown\n", false_positives_N);
	printf("False Negatives   (%8d) dark red\n", false_negatives);
	printf("False Negatives_N (%8d) red\n", false_negatives_N);

	true_positives += true_positives_N;
	true_negatives += true_negatives_N;
	false_positives += false_positives_N;
	false_negatives += false_negatives_N;

	int gt_positives = true_positives + false_negatives;
	int gt_negatives = true_negatives + false_positives;
	int total = gt_positives + gt_negatives;
	int trues = true_positives + true_negatives;
	int positives = true_positives + false_positives;

	if (total == 0)
		return;

	if (positives > 0)
		precision = (double) true_positives / positives;

	if (gt_positives > 0)
		recall = (double) true_positives / gt_positives;

	accuracy = (double) trues / total;

	printf("Positives         (%8d) (%6.4lf%%)\n", gt_positives, (double) 100 * gt_positives / total);
	printf("Negatives         (%8d) (%6.4lf%%)\n", gt_negatives, (double) 100 * gt_negatives / total);

	printf("\nTP                (%8d)\n", true_positives);
	printf("TN                (%8d)\n", true_negatives);
	printf("FP                (%8d)\n", false_positives);
	printf("FN                (%8d)\n", false_negatives);
	printf("----------------------------\n");
	printf("Total             (%8d)\n", total);

	printf("\nPrecision %9.4lf%% \nRecall    %9.4lf%%\nAccuracy  %9.4lf%%\n\n", 100 * precision, 100 * recall, 100 * accuracy);
}


void
compute_maps_changes(char *gt_dir_path, char *clean_dir_path, char *dirty_dir_path)
{
    DIR *gt_dir;
    struct dirent *gt_content;
    char complete_gt_path[1024], complete_clean_path[1024], complete_dirty_path[1024], complete_save_path[1024];
    int x, y;
	char *save_path = (save_png_path == NULL) ? NULL : complete_save_path;

    if ((gt_dir = opendir(gt_dir_path)) == NULL)
    {
        fprintf(stderr, "Error while opening ground_truth_map_path: %s\n", gt_dir_path);
        return;
    }

    while ((gt_content = readdir(gt_dir)) != NULL)
    {
    	unsigned int pos = 0;

    	if (sscanf(gt_content->d_name, "m%d_%d.map%n", &x, &y, &pos) == 2 && pos == strlen(gt_content->d_name))
    	{
    		if (gt_dir_path[strlen(gt_dir_path) - 1] == '/')
    			sprintf(complete_gt_path, "%s%s", gt_dir_path, gt_content->d_name);
    		else
    			sprintf(complete_gt_path, "%s/%s", gt_dir_path, gt_content->d_name);

    		if (clean_dir_path[strlen(clean_dir_path) - 1] == '/')
    			sprintf(complete_clean_path, "%s%s", clean_dir_path, gt_content->d_name);
    		else
    			sprintf(complete_clean_path, "%s/%s", clean_dir_path, gt_content->d_name);

    		if (dirty_dir_path[strlen(dirty_dir_path) - 1] == '/')
    			sprintf(complete_dirty_path, "%s%s", dirty_dir_path, gt_content->d_name);
    		else
    			sprintf(complete_dirty_path, "%s/%s", dirty_dir_path, gt_content->d_name);

			if (save_path)
			{
	    		if (save_png_path[strlen(save_png_path) - 1] == '/')
	    			sprintf(save_path, "%s%s.png", save_png_path, gt_content->d_name);
	    		else
	    			sprintf(save_path, "%s/%s.png", save_png_path, gt_content->d_name);
			}

    		compute_one_map_changes(complete_gt_path, complete_clean_path, complete_dirty_path, save_path);
    	}
    }
    closedir(gt_dir);
}


void
usage(char *program)
{
	fprintf(stderr, "\nUsage: %s <ground_truth_map_path> <clean_map_path> <dirty_map_path> -save <save_png_path> "
			"-poses <poses_file> -from <x> <y> -to <x> <y> -radius <meters> -show -save_complete <save_png_path>\n\n", program);
}


void
exit_error(char const *error_msg, int arg_num, char **argv)
{
	fprintf(stderr, error_msg, arg_num, argv[arg_num]);
	usage(argv[0]);
	exit(-1);
}


int
read_poses(char *filename)
{
	FILE *poses_file = fopen(filename, "r");
	int line_num = 0;
    char *line = NULL;
    size_t line_len = 0;
    char non_whitespace;
    carmen_point_t pose;
    double timestamp;
    bool from_reached = (from_pose.x == 0.0);
    bool to_reached = false;

	if (poses_file == NULL)
		return 0;

	while (getline(&line, &line_len, poses_file) != -1)
	{
		line_num++;
		if (sscanf(line, " %c", &non_whitespace) != 1)
			continue;

		if (sscanf(line, "%lf %lf %lf %lf", &pose.x, &pose.y, &pose.theta, &timestamp) != 4)
		{
			fprintf(stderr, "\nPoses file format error on line %d: %s\n", line_num, line);
			return 0;
		}

		from_reached |= (DIST2D(pose, from_pose) <= radius);
		to_reached |= ((DIST2D(pose, to_pose) <= radius) && (to_pose.x != 0.0));

		if (from_reached && !to_reached)
			poses_array.push_back(pose);
	}
	free(line);
	fclose(poses_file);

	return line_num;
}


void
read_parameters(int argc, char **argv)
{
	int arg_num = 0;

	for (int i = 4; i < argc; i++)
	{
		if (strcmp(argv[i], "-save") == 0)
		{
			i++;
			if (i == argc)
				exit_error("\nPathname expected after [%d]: %s\n", i - 1, argv);
			if (save_png_path)
				exit_error("\nOnly one -save option allowed [%d]: %s\n", i, argv);
			save_png_path = argv[i];
		}
		else if (strcmp(argv[i], "-poses") == 0)
		{
			i++;
			if (i == argc)
				exit_error("\nFilename expected after [%d]: %s\n", i - 1, argv);
			if (poses_filename)
				exit_error("\nOnly one -poses option allowed [%d]: %s\n", i, argv);
			poses_filename = argv[i];
			arg_num = i;
		}
		else if (strcmp(argv[i], "-from") == 0)
		{
			i += 2;
			if (i >= argc)
				exit_error("\nCoordinates <x> <y> expected after [%d]: %s\n", i - 2, argv);
			if (from_pose.x != 0.0)
				exit_error("\nOnly one -from option allowed [%d]: %s\n", i - 1, argv);
			char *px, *py;
			from_pose.x = strtod(argv[i - 1], &px);
			from_pose.y = strtod(argv[i], &py);
			if ((*px) != 0 || (*py) != 0)
				exit_error("\nInvalid coordinates <x> <y> [%d]: %s\n", i - ((*px) != 0), argv);
		}
		else if (strcmp(argv[i], "-to") == 0)
		{
			i += 2;
			if (i >= argc)
				exit_error("\nCoordinates <x> <y> expected after [%d]: %s\n", i - 2, argv);
			if (to_pose.x != 0.0)
				exit_error("\nOnly one -to option allowed [%d]: %s\n", i - 1, argv);
			char *px, *py;
			to_pose.x = strtod(argv[i - 1], &px);
			to_pose.y = strtod(argv[i], &py);
			if ((*px) != 0 || (*py) != 0)
				exit_error("\nInvalid coordinates <x> <y> [%d]: %s\n", i - ((*px) != 0), argv);
		}
		else if (strcmp(argv[i], "-radius") == 0)
		{
			i++;
			if (i == argc)
				exit_error("\nDistance (meters) expected after [%d]: %s\n", i - 1, argv);
			char *p;
			radius = strtod(argv[i], &p);
			if (radius <= 0.0 || (*p) != 0)
				exit_error("\nRadius must have a positive value [%d]: %s\n", i, argv);
		}
		else if (strcmp(argv[i], "-show") == 0)
		{
			show_map_changes = true;
		}
		else if (strcmp(argv[i], "-save_complete") == 0)
		{
			i++;
			if (i == argc)
				exit_error("\nPathname expected after [%d]: %s\n", i - 1, argv);
			if (save_complete_map_path)
				exit_error("\nOnly one -save option allowed [%d]: %s\n", i, argv);
			save_complete_map_path = (char *) malloc (sizeof(char) * 1024);
			sprintf(save_complete_map_path, "%s/%s", argv[i], "complete_map_changes.png");
		}
		else if (strcmp(argv[i], "-show_complete") == 0)
		{
			show_complete_map = true;
		}
		else
			exit_error("\nInvalid option [%d]: %s\n", i, argv);
	}
	if (poses_filename && read_poses(poses_filename) == 0)
		exit_error("\nCould not read poses from file [%d]: %s\n", arg_num, argv);
}


int
main(int argc, char **argv)
{
	if (argc < 4)
		exit_error("", argc, argv);

	read_parameters(argc, argv);

    compute_maps_changes(argv[1], argv[2], argv[3]);

    compute_metrics();

	if (save_complete_map_path)
		imwrite(save_complete_map_path, full_map);

    return 0;
}

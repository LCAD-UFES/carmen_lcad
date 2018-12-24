#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <carmen/carmen.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

#define rrt_obstacle_probability_threshold 0.5			 // carmen-ford-escape.ini
#define IS_OBSTACLE rrt_obstacle_probability_threshold   // Minimum probability of a map cell being an obstacle

int true_positives = 0;
int true_negatives = 0;
int false_positives = 0;
int false_negatives = 0;

char *save_png_path = NULL;
char *poses_filename = NULL;
vector<carmen_point_t> poses_array;
carmen_point_t from_pose = {0.0, 0.0, 0.0};
carmen_point_t to_pose = {0.0, 0.0, 0.0};
double radius = 5.0;
bool show_map_diff = false;


int
str_ends_with(const char * str, const char * suffix)          // Returns 1 if string ends with suffix
{
  if( str == NULL || suffix == NULL )
    return 0;

  size_t str_len = strlen(str);
  size_t suffix_len = strlen(suffix);

  if(suffix_len > str_len)
    return 0;

  return (0 == strncmp((str + str_len - suffix_len), suffix, suffix_len));
}


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
transform_map_to_image(double gt_cell, double map_cell)
{
	if (gt_cell >= 0.0 && map_cell >= 0.0)
	{
		if (gt_cell >  IS_OBSTACLE && map_cell >  IS_OBSTACLE) // true_positives
			return cv::Vec3b(0,0,0);
		if (gt_cell <= IS_OBSTACLE && map_cell <= IS_OBSTACLE) // true_negatives
			return cv::Vec3b(255,255,255);
		if (gt_cell <= IS_OBSTACLE && map_cell >  IS_OBSTACLE) // false_positives
			return cv::Vec3b(0,0,255);
		if (gt_cell >  IS_OBSTACLE && map_cell <= IS_OBSTACLE) // false_negatives
			return cv::Vec3b(0,175,0); // (31,172,255)
	}

	return cv::Vec3b(255, 144, 30);
}


void
compute_map_diff(char *gt_path, char *map_path, char *save_path)
{
	carmen_map_t gt_map, map;

	if (carmen_map_read_gridmap_chunk(gt_path, &gt_map) == 0)
	{
		printf("Opening gt_map: %s\n", gt_path);

		if (carmen_map_read_gridmap_chunk(map_path, &map) == 0)
		{
			printf("Opening map:    %s\n", map_path);
			char *filename = strrchr(gt_path, '/') + 1;
			sscanf(filename, "m%lf_%lf.map", &gt_map.config.x_origin, &gt_map.config.y_origin);

			cv::Mat img = cv::Mat(cv::Size(gt_map.config.x_size, gt_map.config.y_size), CV_8UC3);
			cv::Mat shade = get_area_of_interest_from_poses_array(gt_map.config, radius);

			for (int x = 0; x < gt_map.config.x_size; x++)
			{
				for (int y = 0; y < gt_map.config.y_size; y++)
				{
					cv::Vec3b cell_color = transform_map_to_image(gt_map.map[x][y], map.map[x][y]);
					img.at<cv::Vec3b>(cv::Point(x, gt_map.config.y_size - 1 - y)) = cell_color;

					if (shade.at<cv::Vec3b>(cv::Point(x, gt_map.config.y_size - 1 - y)) != cv::Vec3b(1,1,1))
						continue;

					if (gt_map.map[x][y] >= 0.0 && map.map[x][y] >= 0.0)
					{
						true_positives  += (gt_map.map[x][y] >  IS_OBSTACLE && map.map[x][y] >  IS_OBSTACLE);
						true_negatives  += (gt_map.map[x][y] <= IS_OBSTACLE && map.map[x][y] <= IS_OBSTACLE);
						false_positives += (gt_map.map[x][y] <= IS_OBSTACLE && map.map[x][y] >  IS_OBSTACLE);
						false_negatives += (gt_map.map[x][y] >  IS_OBSTACLE && map.map[x][y] <= IS_OBSTACLE);
					}
				}
			}
			carmen_map_free_gridmap(&map);

			img = img / shade;
			resize(img, img, cv::Size(0, 0), 2.5, 2.5, cv::INTER_NEAREST);

			if (save_path)
			{
				imwrite(save_path, img);
			}

			if (show_map_diff)
			{
				cv::namedWindow(filename);
				cv::moveWindow(filename, 10, 10);
				imshow(filename, img);
				cv::waitKey(0);
				cv::destroyWindow(filename);
			}
		}
		else
			fprintf(stderr, "Could not read map: %s\n", map_path);

		carmen_map_free_gridmap(&gt_map);
	}
	else
		fprintf(stderr, "Could not read gt_map: %s\n", map_path);
}


void
compute_metrics()
{
	double precision = 0.0;
	double recall    = 0.0;
	double accuracy  = 0.0;

	int gt_positives = true_positives + false_negatives;
	int gt_negatives = true_negatives + false_positives;
	int total = gt_positives + gt_negatives;
	int trues = true_positives + true_negatives;
	int positives = true_positives + false_positives;

	printf("True Positives  (%8d)\n", true_positives);
	printf("True Negatives  (%8d)\n", true_negatives);
	printf("False Positives (%8d)\n", false_positives);
	printf("False Negatives (%8d)\n", false_negatives);

	if (total == 0)
		return;

	if (positives > 0)
		precision = (double) true_positives / positives;

	if (gt_positives > 0)
		recall = (double) true_positives / gt_positives;

	accuracy = (double) trues / total;

	printf("Positives %9d (%6.2lf%%)\n", gt_positives, (double) 100 * gt_positives / total);
	printf("Negatives %9d (%6.2lf%%)\n", gt_negatives, (double) 100 * gt_negatives / total);
	printf("Precision %9lf \nRecall    %9lf\nAccuracy  %9lf\n", precision, recall, accuracy);
}


void
compute_maps_difference(char *gt_dir_path, char *map_dir_path)
{
    DIR *gt_dir;
    struct dirent *gt_content;
    char complete_gt_path[1024], complete_path[1024], complete_save_path[1024];
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

    		if (map_dir_path[strlen(map_dir_path) - 1] == '/')
    			sprintf(complete_path, "%s%s", map_dir_path, gt_content->d_name);
    		else
    			sprintf(complete_path, "%s/%s", map_dir_path, gt_content->d_name);

			if (save_path)
			{
	    		if (save_png_path[strlen(save_png_path) - 1] == '/')
	    			sprintf(save_path, "%s%s.png", save_png_path, gt_content->d_name);
	    		else
	    			sprintf(save_path, "%s/%s.png", save_png_path, gt_content->d_name);
			}

    		compute_map_diff(complete_gt_path, complete_path, save_path);
    	}
    }
    closedir(gt_dir);
}


void
usage(char *program)
{
	fprintf(stderr, "\nUsage: %s <ground_truth_map_path> <map_path> -save <save_png_path> "
			"-poses <poses_file> -from <x> <y> -to <x> <y> -radius <meters> -show\n\n", program);
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

	for (int i = 3; i < argc; i++)
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
			show_map_diff = true;
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
	if (argc < 3)
		exit_error("", argc, argv);

	read_parameters(argc, argv);

    compute_maps_difference(argv[1], argv[2]);

    compute_metrics();

    return 0;
}

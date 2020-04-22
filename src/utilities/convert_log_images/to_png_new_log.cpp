#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <locale.h>

using namespace cv;


int
process_bumblebee(FILE *f, char *dir, int camera_side, bool show_image, char* log_path)
{
	int w, h, img_size, is_rect;
	char name[1024], host[64], path[256], suffix[128], timestamp[64], local_timestamp[64];

	fscanf(f, "\n%s", name);
	fscanf(f, "\n%s", suffix);
	fscanf(f, "%d %d", &w, &h);
	fscanf(f, "%d %d", &img_size, &is_rect);
	fscanf(f, "%s %s %s", timestamp, host, local_timestamp);

	if (log_path == NULL)
		sprintf(path, "%s", suffix);
	else
		sprintf(path, "%s%s", log_path, suffix);

	if (w < 300 || h < 300 || w > 10000 || h > 10000)
	{
		fprintf(stderr, "Invalid h: %d and w: %d name: %s img_size: %d\n", h, w, name, img_size);
		return 0;
	}

	FILE *image_file = fopen(path, "rb");

	if (image_file == NULL)
		exit(printf("ERROR: file '%s' not found!\n", path));

	printf ("Saving image: %s%s\n", dir, timestamp);

	static unsigned char *raw_left = (unsigned char*) malloc (img_size * sizeof(unsigned char));
	static unsigned char *raw_right = (unsigned char*) malloc (img_size * sizeof(unsigned char));
	fread(raw_left, img_size, sizeof(unsigned char), image_file);
	fread(raw_right, img_size, sizeof(unsigned char), image_file);

	Mat img_l;
	Mat img_r;

	if (camera_side == 0 || camera_side == INT_MAX)
	{
		img_l = Mat(h, w, CV_8UC3, raw_left, 0);
		cvtColor(img_l, img_l, COLOR_RGB2BGR);

		char out_name_l[1024];
		sprintf(out_name_l, "%s/%s-l.png", dir, timestamp);
		imwrite(out_name_l, img_l, {CV_IMWRITE_PNG_COMPRESSION, 9});
	}

	if (camera_side == 1 || camera_side == INT_MAX)
	{
		img_r = Mat(h, w, CV_8UC3, raw_right, 0);
		cvtColor(img_r, img_r, COLOR_RGB2BGR);

		char out_name_r[1024];
		sprintf(out_name_r, "%s/%s-r.png", dir, timestamp);
		imwrite(out_name_r, img_r, {CV_IMWRITE_PNG_COMPRESSION, 9});
	}

	if (show_image)
	{
		Mat display_image;
		if (camera_side == 0)
			display_image = img_l;
		else if (camera_side == 1)
			display_image = img_r;
		else
			hconcat(img_l, img_r, display_image);

		imshow("Image", display_image);
		waitKey(1);
	}

	fclose(image_file);
	return 1;
}


int
find_side_arg(int argc, char **argv)
{
	int side = INT_MAX;

    for(int i = 0; i < argc; ++i)
    {
        if(!argv[i])
        	continue;

        if(0 == strcmp(argv[i], "-side"))
        {
        	if  (argc < i + 1)
        	{
        		printf("Wrong number of parameters!\n Must be -\"side 0\" for Left side or \"-side 1\" for Right side.\n");
        		exit (0);
        	}
        	side = atoi(argv[i+1]);

        	if  (side < 0 || side > 2)
        	{
        		printf("Wrong Camera Side: %d  Must be -\"side 0\" for Left side or \"-side 1\" for Right side.\n", side);
        		exit (0);
        	}
        }
    }
    return side;
}


void
find_log_path(int argc, char **argv, char* &log_path)
{
    for(int i = 0; i < argc; ++i)
    {
        if(!argv[i])
        	continue;

        if(0 == strcmp(argv[i], "-path"))
        {
        	if  (argc < i + 1)
        	{
        		printf("Wrong number of parameters!\n Must be -\"side 0\" for Left side or \"-side 1\" for Right side.\n");
        		exit (0);
        	}
        	log_path = argv[i + 1];

			return;
        }
    }
}


bool
find_show_arg(int argc, char **argv)
{
    for(int i = 0; i < argc; ++i)
    {
        if(!argv[i])
        	continue;

        if(0 == strcmp(argv[i], "-show"))
        	return (true);
    }
    return (false);
}


int
main(int argc, char **argv)
{
	char* log_path = NULL;

	setlocale(LC_ALL, NULL);

	if (argc < 3)
	{
		printf("Use %s <bumbs.txt> <out-dir>\n", argv[0]);
		printf("Optional params: -show -side <0 or 1> -path <log_path> (only to new logs)\n");
		return 0;
	}

	int status = mkdir(argv[2], 0777);
	if (status == -1)
		printf("Warning: Directory %s already exists\n", argv[2]);
	else if (status != 0)
		exit(printf("ERROR: Could not create directory '%s'\n", argv[2]));

	int camera_side = find_side_arg(argc, argv);
	bool show_image = find_show_arg(argc, argv);
	find_log_path(argc, argv, log_path);


	FILE *f = fopen(argv[1], "r");

	if (f == NULL)
		exit(printf("Unable to open file '%s'\n", argv[1]));

	while (!feof(f))
	{
		process_bumblebee(f, argv[2], camera_side, show_image, log_path);
	}

	fclose(f);
	return 0;
}

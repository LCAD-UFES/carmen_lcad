#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <locale.h>
#include "libdeeplab.h"

using namespace cv;

unsigned int deeplab_semantic_color_map[19][3] =               // Values to map semantic classes to colors
{                                                              // to generate semantic color image
		{128,  64, 128},
		{244,  35, 232},
		{70,   70,  70},
		{102, 102, 156},
		{190, 153, 153},
		{153, 153, 153},
		{250, 170,  30},
		{220, 220,   0},
		{107, 142,  35},
		{152, 251, 152},
		{70,  130, 180},
		{220,  20,  60},
		{255,   0,   0},
		{0,     0, 142},
		{0,     0,  70},
		{0,    60, 100},
		{0,    80, 100},
		{0,     0, 230},
		{119,  11,  32},
};


int
process_bumblebee(FILE *f, char *dir, int camera_side, bool show_image, bool save_colour_map)
{
	int w, h, img_size, is_rect;
	char name[1024], host[64], path[1024], timestamp[64], local_timestamp[64];

	fscanf(f, "\n%s", name);
	fscanf(f, "\n%s", path);
	fscanf(f, "%d %d", &w, &h);
	fscanf(f, "%d %d", &img_size, &is_rect);
	fscanf(f, "%s %s %s", timestamp, host, local_timestamp);

	FILE *image_file = fopen(path, "rb");

	if (image_file == NULL)
	{
		printf("ERROR: file '%s' not found!\n", path);
		return 0;
	}

	static unsigned char *raw_left = (unsigned char*) malloc (img_size * sizeof(unsigned char));
	static unsigned char *raw_right = (unsigned char*) malloc (img_size * sizeof(unsigned char));
	fread(raw_left, img_size, sizeof(unsigned char), image_file);
	fread(raw_right, img_size, sizeof(unsigned char), image_file);


	unsigned char colored_map_l[w*h][3];
	unsigned char colored_map_r[w*h][3];

	Mat img_l, img_r, semantic_cv_l, semantic_cv_r;

	printf ("Saving image: %s/%s\n", dir, timestamp);

	if (camera_side == 0 || camera_side == INT_MAX)
	{
		unsigned char *semantic_left = process_image(w, h, raw_left);

		char out_name_l[1024];
		sprintf(out_name_l, "%s/%s-l.segmap", dir, timestamp);

		FILE *segmap_file = fopen (out_name_l, "wb");
		fwrite (semantic_left, sizeof(unsigned char), (w * h), segmap_file);
		fclose (segmap_file);

		for (int row = 0; row < h; row++)
			for (int col = 0; col < w; col++)
			{
					colored_map_l[col + (row * w)][0] = deeplab_semantic_color_map[semantic_left[col + (row * w)]][0];
					colored_map_l[col + (row * w)][1] = deeplab_semantic_color_map[semantic_left[col + (row * w)]][1];
					colored_map_l[col + (row * w)][2] = deeplab_semantic_color_map[semantic_left[col + (row * w)]][2];
			}
		semantic_cv_l = Mat(h, w, CV_8UC3, colored_map_l, 0);

		if (save_colour_map)
		{
			sprintf(out_name_l, "%s/%s-l.png", dir, timestamp);
			imwrite(out_name_l, semantic_cv_l, {CV_IMWRITE_PNG_COMPRESSION, 9});
		}
	}

	if (camera_side == 1 || camera_side == INT_MAX)
	{
		unsigned char *semantic_right = process_image(w, h, raw_right);

		char out_name_r[1024];
		sprintf(out_name_r, "%s/%s-r.segmap", dir, timestamp);

		FILE *segmap_file = fopen (out_name_r, "wb");
		fwrite (semantic_right, sizeof(unsigned char), (w * h), segmap_file);
		fclose (segmap_file);

		for (int row = 0; row < h; row++)
			for (int col = 0; col < w; col++)
			{
					colored_map_r[col + (row * w)][0] = deeplab_semantic_color_map[semantic_right[col + (row * w)]][0];
					colored_map_r[col + (row * w)][1] = deeplab_semantic_color_map[semantic_right[col + (row * w)]][1];
					colored_map_r[col + (row * w)][2] = deeplab_semantic_color_map[semantic_right[col + (row * w)]][2];
			}
		semantic_cv_r = Mat(h, w, CV_8UC3, colored_map_r, 0);

		if (save_colour_map)
		{
			sprintf(out_name_r, "%s/%s-r.png", dir, timestamp);
			imwrite(out_name_r, semantic_cv_r, {CV_IMWRITE_PNG_COMPRESSION, 9});
		}

	}

	if (show_image)
	{
		Mat display_image;
		if (camera_side == 0 || camera_side == INT_MAX)
		{
			img_l = Mat(h, w, CV_8UC3, raw_left, 0);
			cvtColor(img_l, img_l, COLOR_RGB2BGR);
			hconcat(img_l, semantic_cv_l, display_image);
		}
		else if (camera_side == 1)
		{
			img_r = Mat(h, w, CV_8UC3, raw_right, 0);
			cvtColor(img_r, img_r, COLOR_RGB2BGR);
			hconcat(img_r, semantic_cv_r, display_image);
		}

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


bool
find_save_color_map_arg(int argc, char **argv)
{
    for(int i = 0; i < argc; ++i)
    {
        if(!argv[i])
        	continue;

        if(0 == strcmp(argv[i], "-scolor"))
        	return (true);
    }
    return (false);
}


int
main(int argc, char **argv)
{
	setlocale(LC_ALL, NULL);

	if (argc < 3)
	{
		printf("Use %s <bumbs.txt> <out-dir>\n", argv[0]);
		return 0;
	}

	int status = mkdir(argv[2], 0777);
	if (status == -1)
		printf("Warning: Directory %s already exists.\n", argv[2]);
	else if (status != 0)
		exit(printf("ERROR: Could not create directory '%s'\n", argv[2]));

	initialize_inference_context();

	int camera_side = find_side_arg(argc, argv);
	bool show_image = find_show_arg(argc, argv);
	bool save_colour_map = find_save_color_map_arg(argc, argv);

	FILE *f = fopen(argv[1], "r");

	if (f == NULL)
		exit(printf("Unable to open file '%s'\n", argv[1]));

	while (!feof(f))
	{
		process_bumblebee(f, argv[2], camera_side, show_image, save_colour_map);
	}

	fclose(f);
	return 0;
}

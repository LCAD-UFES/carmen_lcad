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
process_intelbras(FILE *f, char *dir, bool show_image, char* log_filename)
{
	int w, h, img_size, is_rect;
	char name[1024], host[64], path[256], image_path[128], local_timestamp[64];
	int i, camera_id, trash3, trash5, trash6, trash7;
	double timestamp;

	fscanf(f, "%s %d %d %d %lf %s %d %d %d %d %d %d %s", name, &i, &camera_id, &trash3, &timestamp, host, &img_size, &w, &h, &trash5, &trash6, &trash7, local_timestamp);

	int high_level_subdir = ((int) (timestamp / 10000.0)) * 10000.0;
	int low_level_subdir = ((int) (timestamp / 100.0)) * 100.0;

	printf("%d %d %d\n", img_size, w, h);

	sprintf(path, "%s_images/%d/%d/%lf_camera%d_%d.image", log_filename, high_level_subdir, low_level_subdir, timestamp, camera_id, i);

	FILE *image_file = fopen(path, "rb");

	if (image_file == NULL)
	{
		printf("ERROR: file '%s' not found!\n", path);
	} 
	else {

		printf ("Saving image: %s%lf\n", dir, timestamp);

		static unsigned char *raw = (unsigned char*) malloc (img_size * sizeof(unsigned char));
		fread(raw, img_size, sizeof(unsigned char), image_file);

		Mat img;
		img = Mat(h, w, CV_8UC3, raw, 0);
		cvtColor(img, img, COLOR_RGB2BGR);

		char out_name[1024];
		sprintf(out_name, "%s/%lf_camera%d_%d.png", dir, timestamp, camera_id, i);
		imwrite(out_name, img, {CV_IMWRITE_PNG_COMPRESSION, 9});

		if (show_image)
		{
			Mat display_image;
			display_image = img;

			imshow("Image", display_image);
			waitKey(1);
		}


		fclose(image_file);
	}
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

	setlocale(LC_ALL, NULL);

	if (argc < 4)
	{
		printf("Use %s <camera.txt> <out-dir> <log_filename>\n", argv[0]);
		printf("Optional params: -show\n");
		return 0;
	}

	int status = mkdir(argv[2], 0777);
	if (status == -1)
		printf("Warning: Directory %s already exists\n", argv[2]);
	else if (status != 0)
		exit(printf("ERROR: Could not create directory '%s'\n", argv[2]));

	bool show_image = find_show_arg(argc, argv);

	FILE *f = fopen(argv[1], "r");

	if (f == NULL)
		exit(printf("Unable to open file '%s'\n", argv[1]));

	while (!feof(f))
	{
		process_intelbras(f, argv[2], show_image, argv[3]);
	}

	fclose(f);
	return 0;
}

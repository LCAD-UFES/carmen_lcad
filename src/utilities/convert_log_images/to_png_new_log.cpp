
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace cv;


int
process_bumblebee(FILE *f, char *dir)
{
	int i, j, w, h, img_size, is_rect;
	char name[1024], host[64], path[1024];
	uchar r, g, b;
	double timestamp;
	double local_timestamp;
	static uchar *raw_left = NULL;
	static uchar *raw_right = NULL;

	fscanf(f, "\n%s", name);
	fscanf(f, "\n%s", path);
	fscanf(f, "%d %d", &w, &h);
	fscanf(f, "%d %d", &img_size, &is_rect);

	if (w < 300 || h < 300 || w > 10000 || h > 10000)
	{
		fprintf(stderr, "Invalid h: %d and w: %d name: %s img_size: %d\n", h, w, name, img_size);
		return 0;
	}

	FILE *image_file = fopen(path, "r");

	if (image_file == NULL)
		exit(printf("ERROR: file '%s' not found!\n", path));

	if(raw_left == NULL) raw_left = (unsigned char*) malloc (img_size * sizeof(unsigned char));
	if(raw_right == NULL) raw_right = (unsigned char*) malloc (img_size * sizeof(unsigned char));

    fread(raw_left, img_size, sizeof(unsigned char), image_file);
    fread(raw_right, img_size, sizeof(unsigned char), image_file);

	fclose(image_file);

	Mat img_r(h, w, CV_8UC3);
	Mat img_l(h, w, CV_8UC3);

	for(i = 0; i < h; i++)
	{
		for(j = 0; j < w; j++)
		{
			r = raw_left[3 * (i * w + j) + 0];
			g = raw_left[3 * (i * w + j) + 1];
			b = raw_left[3 * (i * w + j) + 2];

			img_l.data[3 * (i * w + j) + 0] = b;
			img_l.data[3 * (i * w + j) + 1] = g;
			img_l.data[3 * (i * w + j) + 2] = r;
		}
	}

	for(i = 0; i < h; i++)
	{
		for(j = 0; j < w; j++)
		{
			r = raw_right[3 * (i * w + j) + 0];
			g = raw_right[3 * (i * w + j) + 1];
			b = raw_right[3 * (i * w + j) + 2];

			img_r.data[3 * (i * w + j) + 0] = b;
			img_r.data[3 * (i * w + j) + 1] = g;
			img_r.data[3 * (i * w + j) + 2] = r;
		}
	}

	imshow("img_r", img_r);
	imshow("img_l", img_l);
	waitKey(1);

	fscanf(f, "%lf %s %lf", &timestamp, host, &local_timestamp);

	char outnamel[1024];
	char outnamer[1024];
	sprintf(outnamer, "%s/%f-r.png", dir, timestamp);
	imwrite(outnamer, img_r);
	sprintf(outnamel, "%s/%f-l.png", dir, timestamp);
	imwrite(outnamel, img_l);

	return 1;
}


int
main(int argc, char **argv)
{
	if (argc < 4)
	{
		printf("Use %s <bumbs.txt> <out-dir>\n", argv[0]);
		return 0;
	}

	FILE *f = fopen(argv[1], "r");

	if (f == NULL)
		exit(printf("Unable to open file '%s'\n", argv[1]));

	while (!feof(f))
	{
		process_bumblebee(f, argv[2]);
	}

	fclose(f);
	return 0;
}



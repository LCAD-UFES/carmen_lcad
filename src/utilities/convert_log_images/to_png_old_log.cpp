
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace cv;


#define HEX_TO_BYTE(hi, lo) (hi << 4 | lo)
#define HEX_TO_SHORT(fourth, third, second, first) ( fourth << 12 | (third << 8 | (second << 4 | first)))


#define HEX_TO_RGB_BYTE(hi, lo) (hi << 4 | lo)
#define GETINDEX(a) isalpha(a) ? a - 'a' + 10 : a - '0'


uchar
read_pixel(FILE *f)
{
	uchar p, r, hi, lo;

	do { r = fgetc(f); } while (r == ' ');
	hi = GETINDEX(r);

	do { r = fgetc(f); } while (r == ' ');
	lo = GETINDEX(r);

	p = HEX_TO_RGB_BYTE(hi, lo);

	return p;
}


int
process_bumblebee(FILE *f, char *dir, FILE *outfile)
{
	int i, w, h, img_size, is_rect;
	char name[64], host[64];
	uchar r, g, b;
	double timestamp;
	double local_timestamp;

	fscanf(f, "\n%s", name);
	fscanf(f, "%d %d", &w, &h);
	fscanf(f, "%d %d", &img_size, &is_rect);

	if (w < 300 || h < 300 || w > 10000 || h > 10000)
	{
		fprintf(stderr, "Invalid h: %d and w: %d name: %s img_size: %d\n", h, w, name, img_size);
		return 0;
	}

	//printf("%d %d %d\n", h, w, img_size);
	//getchar();

	Mat img_r(h, w, CV_8UC3);
	Mat img_l(h, w, CV_8UC3);

	for(i = 0; i < img_size / 3; i++)
	{
		//printf("RIGHT %d de %d\n", i, img_size / 3);

		r = read_pixel(f);
		g = read_pixel(f);
		b = read_pixel(f);

		img_r.data[3 * i + 0] = b;
		img_r.data[3 * i + 1] = g;
		img_r.data[3 * i + 2] = r;
	}

	for(i = 0; i < img_size / 3; i++)
	{
		//printf("LEFT %d de %d\n", i, img_size / 3);

		r = read_pixel(f);
		g = read_pixel(f);
		b = read_pixel(f);

		img_l.data[3 * i + 0] = b;
		img_l.data[3 * i + 1] = g;
		img_l.data[3 * i + 2] = r;
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

	fprintf(outfile, "%lf %s %s\n", timestamp, outnamel, outnamer);

	return 1;
}


int
main(int argc, char **argv)
{
	if (argc < 4)
	{
		printf("Use %s <bumbs.txt> <out-dir> <out-file>\n", argv[0]);
		return 0;
	}

	FILE *f = fopen(argv[1], "r");
	FILE *g = fopen(argv[3], "w");

	if (f == NULL)
		exit(printf("Unable to open file '%s'\n", argv[1]));

	if (g == NULL)
		exit(printf("Unable to open file '%s'\n", argv[3]));

	while (!feof(f))
	{
		//printf("LENDO\n");
		process_bumblebee(f, argv[2], g);
	}

	fclose(f);
	fclose(g);
	return 0;
}



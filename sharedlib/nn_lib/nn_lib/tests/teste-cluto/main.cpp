#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "cluto.h"


void
initialize_mtx(IplImage *img, float *mtx)
{
	int i, j, k, mtx_pos, img_pos;
	
	for(i = 0; i < img->height; i++)
	{
		for(j = 0; j < img->width; j++)
		{
			mtx_pos = img->nChannels * (i * img->width + j);
			img_pos = img->nChannels * (i * img->width + j);
			
			for(k = 0; k < img->nChannels; k++)
			{
				if (img->imageData[img_pos + 0] > 0)
					mtx[mtx_pos + k] = (float) img->imageData[img_pos + k] / (float) img->imageData[img_pos + 0];
				else
					mtx[mtx_pos + k] = 0;
			}
		}
	}
}


void
generate_output_image_from_cluster_ids(IplImage *output_img, int *sample_cluster_id, int num_clusters, 	CvScalar *color_table)
{
	int i, j, img_pos, mtx_pos, cluster_id;

	for(i = 0; i < output_img->height; i++)
	{
		for(j = 0; j < output_img->width; j++)
		{
			mtx_pos = i * output_img->width + j;
			img_pos = 3 * (i * output_img->width + j);
			
			cluster_id = sample_cluster_id[mtx_pos];
			
			output_img->imageData[img_pos + 0] = color_table[cluster_id].val[0];
			output_img->imageData[img_pos + 1] = color_table[cluster_id].val[1];
			output_img->imageData[img_pos + 2] = color_table[cluster_id].val[2];			
		}
	}
}


void
initialize_color_table(CvScalar *color_table)
{
	int i;
	
	color_table[0] = cvScalar(255, 0, 0, 0);
	color_table[1] = cvScalar(0, 255, 0, 0);
	color_table[2] = cvScalar(0, 0, 255, 0);
	color_table[3] = cvScalar(255, 255, 0, 0);
	color_table[4] = cvScalar(255, 0, 255, 0);
	color_table[5] = cvScalar(255, 255, 255, 0);
	color_table[6] = cvScalar(127, 0, 0, 0);
	color_table[7] = cvScalar(0, 127, 0, 0);
	color_table[8] = cvScalar(0, 0, 127, 0);
	color_table[9] = cvScalar(127, 127, 0, 0);
	color_table[10] = cvScalar(127, 0, 127, 0);
	color_table[11] = cvScalar(127, 127, 127, 0);
	color_table[12] = cvScalar(127, 255, 0, 0);
	color_table[13] = cvScalar(127, 0, 255, 0);
	color_table[14] = cvScalar(255, 127, 0, 0);
	color_table[15] = cvScalar(255, 0, 127, 0);
	color_table[16] = cvScalar(0, 255, 127, 0);
	color_table[17] = cvScalar(0, 127, 255, 0);
	color_table[18] = cvScalar(255, 255, 127, 0);
	color_table[19] = cvScalar(255, 127, 255, 0);
	color_table[20] = cvScalar(127, 255, 255, 0);
	
	// ... continuar ate 50
}

int
main(int argc, char **argv)
{
	int i, num_samples, num_features, num_clusters;
	
	if (argc < 4)
		exit(printf("Use %s <img> <output> <num-clusters>\n", argv[0]));
	
	IplImage *img = cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR);
	
	if (img == NULL)
		exit(printf("Image '%s' not found\n", argv[1]));
	
	IplImage *output_img = cvCreateImage(cvSize(img->width, img->height), img->depth, img->nChannels);
	
	num_samples = img->height * img->width;
	num_features = img->nChannels;
	num_clusters = atoi(argv[3]);
	
	float mtx [num_samples * num_features];
	int sample_cluster_id[num_samples];
	CvScalar color_table[num_clusters];

	printf("Initializing mtx...\n");
	initialize_mtx(img, mtx);
	
	initialize_color_table(color_table);
	
	srand(time(NULL));

	printf("Clustering...\n");	
	CLUTO_VP_ClusterRB (num_samples, num_features, NULL, NULL, mtx,
		CLUTO_SIM_COSINE, CLUTO_CLFUN_I1, CLUTO_ROWMODEL_NONE, CLUTO_COLMODEL_NONE, 1.0,
		10, 10, rand(), CLUTO_CSTYPE_BESTFIRST, 1,
		0, num_clusters, sample_cluster_id);

	printf("Generating output image...\n");
	generate_output_image_from_cluster_ids(output_img, sample_cluster_id, num_clusters, color_table);
	cvSaveImage(argv[2], output_img, NULL);	
	
	printf("Finish!\n");
	return 0;
}

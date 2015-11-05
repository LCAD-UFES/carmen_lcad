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
				mtx[mtx_pos + k] = (float) img->imageData[img_pos + k];
			}
		}
	}
}


void
initialize_modified_mtx(IplImage *img, float *mtx)
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
	
	if (argc < 2)
		exit(printf("Use %s <num-clusters>\n", argv[0]));
	
	num_clusters = atoi(argv[3]);
	
	IplImage *frame, *output_image, *output_image_modified;
	
	CvScalar color_table[50];
	
	CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);

	if (capture == NULL)
		exit(printf("Error: Capture initialization failed! Please check if the camera is turned on!\n"));
	
	frame = cvQueryFrame(capture);
	
	printf("Initialize\n");
	initialize_color_table(color_table);
	
	num_samples = frame->height * frame->width;
	num_features = frame->nChannels;
	
	printf("creating mtxs\n", (num_samples * num_features * 4));
	
	float mtx[num_samples * num_features];
	float mtx_modified[num_samples * num_features];
	int sample_cluster_id[num_samples];
	int sample_cluster_id_modified[num_samples];
	
	//float *mtx, *mtx_modified;
	//int *sample_cluster_id, *sample_cluster_id_modified;
	//
	//mtx = (float*) calloc (num_samples * num_features, sizeof(float));
	//mtx_modified = (float*) calloc (num_samples * num_features, sizeof(float));			
	//sample_cluster_id = (int*) calloc (num_samples, sizeof(int));
	//sample_cluster_id_modified = (int*) calloc (num_samples, sizeof(int));
	
	printf("creating output images\n");
	
	output_image = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, frame->nChannels);
	output_image_modified = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, frame->nChannels);
			
	srand(time(NULL));
	
	while (1)
	{
		frame = cvQueryFrame(capture);
		
		printf("init mtxs\n");
		initialize_mtx(frame, mtx);
		initialize_modified_mtx(frame, mtx_modified);
		
		printf("clustering - 1\n");
		CLUTO_VP_ClusterRB (num_samples, num_features, NULL, NULL, mtx,
			CLUTO_SIM_COSINE, CLUTO_CLFUN_I1, CLUTO_ROWMODEL_NONE, CLUTO_COLMODEL_NONE, 1.0,
			3, 3, rand(), CLUTO_CSTYPE_BESTFIRST, 1,
			0, num_clusters, sample_cluster_id);

		printf("clustering - 2\n");			
		CLUTO_VP_ClusterRB (num_samples, num_features, NULL, NULL, mtx_modified,
			CLUTO_SIM_COSINE, CLUTO_CLFUN_I1, CLUTO_ROWMODEL_NONE, CLUTO_COLMODEL_NONE, 1.0,
			3, 3, rand(), CLUTO_CSTYPE_BESTFIRST, 1,
			0, num_clusters, sample_cluster_id_modified);

		printf("Oi\n");
		generate_output_image_from_cluster_ids(output_image, sample_cluster_id, num_clusters, color_table);
		printf("Oi - 2\n");
		generate_output_image_from_cluster_ids(output_image_modified, sample_cluster_id_modified, num_clusters, color_table);
		
		cvShowImage("frame", frame);
		cvShowImage("common color", output_image);
		cvShowImage("divided color", output_image_modified);
		
		if ((cvWaitKey(10) & 255) == 27) break;
	}
	
	printf("Finish!\n");
	return 0;
}

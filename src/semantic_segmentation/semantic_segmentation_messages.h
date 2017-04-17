#ifndef CARMEN_SEMANTIC_SEGMENTATION_MESSAGES_H
#define CARMEN_SEMANTIC_SEGMENTATION_MESSAGES_H

typedef struct
{
	int width;
	int height;
	int image_size;
	unsigned char *image;
	double timestamp;
	char *host;
} carmen_semantic_segmentation_image_message;

#define      CARMEN_SEMANTIC_SEGMENTATION_IMAGE_NAME       "carmen_semantic_segmentation_image"
#define      CARMEN_SEMANTIC_SEGMENTATION_IMAGE_FMT        "{int,int,int,<ubyte:3>,double,string}"

#endif

/*********************************************************
 Visual TRACKER Module
*********************************************************/

#ifndef CARMEN_VISUAL_TRACKER_MESSAGES_H
#define CARMEN_VISUAL_TRACKER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int x;
	int y;
	int width;
	int height;
} bounding_box;

typedef struct {
	int	size;
	int width;
	int height;
	int channels;
	unsigned char *image;
	bounding_box rect;
	double timestamp;
	char *host;
} carmen_visual_tracker_train_message;

typedef struct {
	int	size;
	int width;
	int height;
	int channels;
	unsigned char *image;
	double timestamp;
	char *host;
} carmen_visual_tracker_test_message;

//Verificar se eh necessario mudar para ter dados da camera usada
typedef struct {
	bounding_box rect;
	double confidence;
	double timestamp;
	char *host;
} carmen_visual_tracker_output_message;

#define		CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_NAME	"carmen_visual_tracker_train_message"
#define		CARMEN_VISUAL_TRACKER_TRAIN_MESSAGE_FMT		"{int,int,int,int,<ubyte:1>,{int,int,int,int},double,string}"

#define		CARMEN_VISUAL_TRACKER_TEST_MESSAGE_NAME		"carmen_visual_tracker_test_message"
#define		CARMEN_VISUAL_TRACKER_TEST_MESSAGE_FMT		"{int,int,int,int,<ubyte:1>,double,string}"

#define		CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_NAME	"carmen_visual_tracker_output_message"
#define		CARMEN_VISUAL_TRACKER_OUTPUT_MESSAGE_FMT	"{{int,int,int,int},double,double,string}"

#ifdef __cplusplus
}
#endif

#endif



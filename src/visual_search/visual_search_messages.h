/*********************************************************
 Visual Search Module
*********************************************************/

#ifndef CARMEN_VISUAL_SEARCH_MESSAGES_H
#define CARMEN_VISUAL_SEARCH_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

//VISUAL_SEARCH_STATE previously defined as a Macro
#define		VISUAL_SEARCH_STATE int
#define		NON_STARTED 0
#define		WAITING_FOR_TRAINNING 1
#define		TRAINNING_NETWORK 2
#define		RUNNING_NETWORK  4

typedef struct {
	int	x_point;
	int	y_point;
	int	reference_image_size;		/* input_image_width * input_image_height * 3 */
	int forget_last_training;
	unsigned char	*reference_image;
	double		timestamp;
	char		*host;
}	carmen_visual_search_message;

typedef struct {
	int	reference_image_size;		/* input_image_width * input_image_height * 3 */
	unsigned char	*reference_image;
	double		scale;
	double		timestamp;
	char		*host;
}	carmen_visual_search_test_message;

typedef struct {
	int	video_data_size;		/* width * height * 3 */
	int	depth_data_size;		/* width * height */
	unsigned char	*video_data;		/* kinect video data */
	float		*depth_data;		/* kinect depth data */
	double		timestamp;
	char		*host;
}	carmen_visual_search_kinect_test_message;

typedef struct {
	int	x_point;
	int	y_point;
	int	x_saccade_vector;
	int	y_saccade_vector;
	double		measured_scale_factor;
	double		timestamp;
	char		*host;
} 	carmen_visual_search_output_message;

typedef struct {
	VISUAL_SEARCH_STATE	state;
	double		timestamp;
	char		*host;
}	carmen_visual_search_state_change_message;

#define		CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_NAME	"carmen_visual_search_training_message"
#define		CARMEN_VISUAL_SEARCH_TRAINING_MESSAGE_FMT	"{int,int,int,int,<ubyte:3>,double,string}"

#define		CARMEN_VISUAL_SEARCH_TEST_MESSAGE_NAME		"carmen_visual_search_test_message"
#define		CARMEN_VISUAL_SEARCH_TEST_MESSAGE_FMT		"{int,<ubyte:1>,double,double,string}"

#define		CARMEN_VISUAL_SEARCH_KINECT_TEST_MESSAGE_NAME		"carmen_visual_search_kinect_test_message"
#define		CARMEN_VISUAL_SEARCH_KINECT_TEST_MESSAGE_MESSAGE_FMT	"{int,int,<ubyte:1>,<float:2>,double,string}"

#define		CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_NAME	"carmen_visual_search_output_message"
#define		CARMEN_VISUAL_SEARCH_OUTPUT_MESSAGE_FMT		"{int,int,int,int,double,double,string}"

#define		CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_NAME	"carmen_visual_search_state_change_message" 
#define		CARMEN_VISUAL_SEARCH_STATE_CHANGE_MESSAGE_FMT	"{int,double,string}"

#define 	CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_NAME      	"carmen_visual_search_query_message_name"
#define		CARMEN_VISUAL_SEARCH_QUERY_MESSAGE_FMT			CARMEN_VISUAL_SEARCH_TEST_MESSAGE_FMT

#ifdef __cplusplus
}
#endif

#endif



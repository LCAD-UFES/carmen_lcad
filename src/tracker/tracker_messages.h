/*********************************************************
 Visual Search Module
*********************************************************/

#ifndef CARMEN_TRACKER_MESSAGES_H
#define CARMEN_TRACKER_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	carmen_position_t object_position;
	double timestamp;
	char *host;
}	carmen_tracker_position_message;

#define CARMEN_TRACKER_POSITION_MESSAGE_NAME "carmen_tracker_position_message"
#define	CARMEN_TRACKER_POSITION_MESSAGE_FMT	"{{double, double}, double, string}"

typedef struct {
	int reference_points_size;
	carmen_position_t *reference_points;
	int reference_image_size;		/* input_image_width * input_image_height * 3 */
	unsigned char *reference_image;
	double timestamp;
	char *host;
}	carmen_tracker_train_message;

typedef struct {
	int associated_points_size;
	carmen_position_t *associated_points;
	int associated_image_size;		/* input_image_width * input_image_height * 3 */
	unsigned char *associated_image;
	double scale;
	double timestamp;
	char *host;
}	carmen_tracker_test_message;

typedef struct {
	carmen_position_t saccade_point;
	carmen_position_t saccade_vector;
	double measured_scale_factor;
	double measured_confidence;
	double timestamp;
	char *host;
	double width;
	double height;
} 	carmen_tracker_output_message;

typedef 	carmen_default_message carmen_tracker_output_training_message;

#define		CARMEN_TRACKER_TRAINING_MESSAGE_NAME	"carmen_tracker_training_message"
#define		CARMEN_TRACKER_TRAINING_MESSAGE_FMT	"{int,<{double,double}:1>,int,<ubyte:3>,double,string}"

#define		CARMEN_TRACKER_TEST_MESSAGE_NAME		"carmen_tracker_test_message"
#define		CARMEN_TRACKER_TEST_MESSAGE_FMT		"{int,<{double,double}:1>,int,<ubyte:3>,double,double,string}"

#define		CARMEN_TRACKER_OUTPUT_MESSAGE_NAME	"carmen_tracker_output_message"
#define		CARMEN_TRACKER_OUTPUT_MESSAGE_FMT	"{{double,double},{double,double},double,double,double,string,double,double}"

#define 	CARMEN_TRACKER_QUERY_TEST_MESSAGE_NAME	"carmen_tracker_query_test_message_name"
#define		CARMEN_TRACKER_QUERY_TEST_MESSAGE_FMT	CARMEN_TRACKER_TEST_MESSAGE_FMT

#define 	CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_NAME	"carmen_tracker_query_training_message_name"
#define		CARMEN_TRACKER_QUERY_TRAINING_MESSAGE_FMT	CARMEN_TRACKER_TRAINING_MESSAGE_FMT

#define 	CARMEN_TRACKER_OUTPUT_TRAINING_MESSAGE_NAME	"carmen_tracker_output_training_message_name"
#define		CARMEN_TRACKER_OUTPUT_TRAINING_MESSAGE_FMT	CARMEN_DEFAULT_MESSAGE_FMT

#ifdef __cplusplus
}
#endif

#endif



/*********************************************************
 Visual Search Module
*********************************************************/

#ifndef CARMEN_VISUAL_SEARCH_THIN_MESSAGES_H
#define CARMEN_VISUAL_SEARCH_THIN_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int	reference_points_size;
	carmen_position_t *reference_points;
	int	reference_image_size;		/* input_image_width * input_image_height * 3 */
	unsigned char	*reference_image;
	double		timestamp;
	char		*host;
}	carmen_visual_search_thin_train_message;

typedef struct {
	int	associated_points_size;
	carmen_position_t *associated_points;
	int	associated_image_size;		/* input_image_width * input_image_height * 3 */
	unsigned char	*associated_image;
	double		scale;
	double		timestamp;
	char		*host;
}	carmen_visual_search_thin_test_message;

typedef struct {
	carmen_position_t saccade_point;
	carmen_position_t saccade_vector;
	double		measured_scale_factor;
	double		measured_confidence;
	double		timestamp;
	char		*host;
} 	carmen_visual_search_thin_output_message;

typedef 	carmen_default_message carmen_visual_search_thin_output_training_message;

#define		CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_NAME	"carmen_visual_search_thin_training_message"
#define		CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_FMT	"{int,<{double,double}:1>,int,<ubyte:3>,double,string}"

#define		CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_NAME		"carmen_visual_search_thin_test_message"
#define		CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_FMT		"{int,<{double,double}:1>,int,<ubyte:3>,double,double,string}"

#define		CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_NAME	"carmen_visual_search_thin_output_message"
#define		CARMEN_VISUAL_SEARCH_THIN_OUTPUT_MESSAGE_FMT	"{{double,double},{double,double},double,double,double,string}"

#define 	CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_NAME	"carmen_visual_search_thin_query_test_message_name"
#define		CARMEN_VISUAL_SEARCH_THIN_QUERY_TEST_MESSAGE_FMT	CARMEN_VISUAL_SEARCH_THIN_TEST_MESSAGE_FMT

#define 	CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_NAME	"carmen_visual_search_thin_query_training_message_name"
#define		CARMEN_VISUAL_SEARCH_THIN_QUERY_TRAINING_MESSAGE_FMT	CARMEN_VISUAL_SEARCH_THIN_TRAINING_MESSAGE_FMT

#define 	CARMEN_VISUAL_SEARCH_THIN_OUTPUT_TRAINING_MESSAGE_NAME	"carmen_visual_search_thin_output_training_message_name"
#define		CARMEN_VISUAL_SEARCH_THIN_OUTPUT_TRAINING_MESSAGE_FMT	CARMEN_DEFAULT_MESSAGE_FMT

#ifdef __cplusplus
}
#endif

#endif



/*********************************************************
 Visual Search Module
*********************************************************/

#ifndef CARMEN_VERGENCE_MESSAGES_H
#define CARMEN_VERGENCE_MESSAGES_H

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
}	carmen_vergence_train_message;

typedef struct {
	int	associated_points_size;
	carmen_position_t *associated_points;
	int	associated_image_size;		/* input_image_width * input_image_height * 3 */
	unsigned char	*associated_image;
	double		timestamp;
	char		*host;
} 	carmen_vergence_test_message;

typedef struct {
	carmen_position_t vergence_point;
	double		confidence;
	double		timestamp;
	char		*host;
} 	carmen_vergence_test_output_message;

typedef 	carmen_default_message carmen_vergence_train_output_message;

#define		CARMEN_VERGENCE_TRAIN_MESSAGE_NAME	"carmen_vergence_train_message"
#define		CARMEN_VERGENCE_TRAIN_MESSAGE_FMT	"{int,<{double,double}:1>,int,<ubyte:3>,double,string}"

#define		CARMEN_VERGENCE_TEST_MESSAGE_NAME	"carmen_vergence_test_message"
#define		CARMEN_VERGENCE_TEST_MESSAGE_FMT	"{int,<{double,double}:1>,int,<ubyte:3>,double,string}"

#define		CARMEN_VERGENCE_TEST_OUTPUT_MESSAGE_NAME	"carmen_vergence_test_output_message"
#define		CARMEN_VERGENCE_TEST_OUTPUT_MESSAGE_FMT		"{{double,double},double,double,string}"

#define 	CARMEN_VERGENCE_TRAIN_OUTPUT_MESSAGE_NAME	"carmen_vergence_train_output_message"
#define		CARMEN_VERGENCE_TRAIN_OUTPUT_MESSAGE_FMT	CARMEN_DEFAULT_MESSAGE_FMT

#define 	CARMEN_VERGENCE_QUERY_TEST_MESSAGE_NAME		"carmen_vergence_query_test_message"
#define		CARMEN_VERGENCE_QUERY_TEST_MESSAGE_FMT		CARMEN_VERGENCE_TEST_MESSAGE_FMT

#define 	CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_NAME	"carmen_vergence_query_train_message"
#define		CARMEN_VERGENCE_QUERY_TRAIN_MESSAGE_FMT		CARMEN_VERGENCE_TRAIN_MESSAGE_FMT

#ifdef __cplusplus
}
#endif

#endif



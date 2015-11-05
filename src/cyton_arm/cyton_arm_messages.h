#ifndef CARMEN_CYTON_ARM_MESSAGES_H
#define CARMEN_CYTON_ARM_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

#define CARMEN_CYTON_ARM_STATE_QUERY_NAME	"carmen_cyton_arm_state_query"
typedef carmen_default_message carmen_cyton_arm_state_query_message;
#define CARMEN_CYTON_ARM_RESET_NAME			"carmen_cyton_arm_reset"
typedef carmen_default_message carmen_cyton_arm_reset_message;

typedef struct {
	double x, y, z;
	double gripper;
	double timestamp;
	char* host;
} carmen_cyton_arm_point_command_message;

#define CARMEN_CYTON_ARM_POINT_COMMAND_NAME	"carmen_cyton_arm_point_command"
#define CARMEN_CYTON_ARM_POINT_COMMAND_FMT	"{double, double, double, double, double, string}"

typedef struct {
	int num_joints;
	double *joint_angles;
	double timestamp;
	char *host;
} carmen_cyton_arm_joint_command_message;

#define CARMEN_CYTON_ARM_JOINT_COMMAND_NAME	"carmen_cyton_arm_joint_command"
#define CARMEN_CYTON_ARM_JOINT_COMMAND_FMT	"{int, <double:1>, double, string}"

typedef struct {
	int arm_busy;
	double x, y, z;
	double gripper;
	
	int num_joints;
	double *joint_angles;

	double timestamp;
	char *host;
} carmen_cyton_arm_state_message;

#define CARMEN_CYTON_ARM_STATE_NAME	"carmen_cyton_arm_state"
#define CARMEN_CYTON_ARM_STATE_FMT	"{int, double, double, double, double, int, <double:6>, double, string}"

/*
typedef struct {
	int flags;
	int num_joints;
	double *joint_angles;
	double timestamp;
	char* host;
} carmen_cyton_arm_joint_state_message;

#define CARMEN_CYTON_ARM_JOINT_STATE_NAME	"carmen_cyton_arm_joint_state"
#define CARMEN_CYTON_ARM_JOINT_STATE_FMT	"{int, int, <double:1>, double, string}"
*/

#ifdef __cplusplus
}
#endif

#endif

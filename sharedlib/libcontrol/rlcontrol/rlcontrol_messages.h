
#ifndef CARMEN_RL_CONTROL_MESSAGES_H
#define CARMEN_RL_CONTROL_MESSAGES_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct 
{
	double steering;
	double throttle;
	double brake;
	double timestamp;
	char *host;
} carmen_rl_control_message;
  
#define      CARMEN_RL_CONTROL_NAME			"carmen_rl_control_message"
#define      CARMEN_RL_CONTROL_FMT			"{double,double,double,double,string}"

#ifdef __cplusplus
}
#endif

#endif
// @}


#ifndef _CARMEN_UDATMO_NODE_H_
#define _CARMEN_UDATMO_NODE_H_


#ifdef __cplusplus
extern "C"
{
#endif


#include "udatmo_messages.h"


void carmen_udatmo_define_messages(void);


void carmen_udatmo_install_params(int argc, char *argv[]);


void carmen_udatmo_subscribe_messages(void);


#ifdef __cplusplus
}
#endif


#endif

// @}

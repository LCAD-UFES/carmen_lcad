#include <carmen/xsens_mtig_messages.h>

#ifdef __cplusplus
extern "C" 
{
#endif

void
carmen_xsens_mtig_subscribe_message(carmen_xsens_mtig_message *message,
			       carmen_handler_t handler, carmen_subscribe_t subscribe_how);

void
carmen_xsens_mtig_unsubscribe_message(carmen_handler_t handler);

void
carmen_xsens_mtig_define_messages();

#ifdef __cplusplus
}
#endif


// @}


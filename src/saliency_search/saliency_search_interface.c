#include "saliency_search_interface.h"

void
carmen_saliency_search_subscribe_saliency_points(carmen_saliency_search_saliency_points_message *message,
			       carmen_handler_t handler,
			       carmen_subscribe_t subscribe_how)
{
  carmen_subscribe_message(CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_NAME,
		  	  	  	  	   CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_FMT,
                           message, sizeof(carmen_saliency_search_saliency_points_message),
                           handler, subscribe_how);
}


void
carmen_saliency_search_unsubscribe_saliency_points(carmen_handler_t handler)
{
  carmen_unsubscribe_message(CARMEN_SALIENCY_SEARCH_SALIENCY_POINTS_MESSAGE_NAME, handler);
}


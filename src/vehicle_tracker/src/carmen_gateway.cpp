#include "carmen_gateway.h"

/** \brief Input global position message. */
static carmen_localize_ackerman_globalpos_message *globalpos_message;

/** \brief Input virtual scan message. */
static carmen_virtual_scan_message *virtual_scan_message;

/**
 * \brief Input virtual scan message hendler.
 */
static void carmen_localize_ackerman_globalpos_handler(void *msg)
{
	globalpos_message = (carmen_localize_ackerman_globalpos_message*) msg;
}

/**
 * \brief Input virtual scan message hendler.
 */
static void carmen_virtual_scan_handler(void *msg)
{
	virtual_scan_message = (carmen_virtual_scan_message*) msg;
}

void carmen_gateway_init(int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);

	carmen_localize_ackerman_subscribe_globalpos_message(
		NULL,
		carmen_localize_ackerman_globalpos_handler,
		CARMEN_SUBSCRIBE_LATEST
	);

	carmen_virtual_scan_subscribe_message(
		NULL,
		carmen_virtual_scan_handler,
		CARMEN_SUBSCRIBE_LATEST
	);
}

void carmen_gateway_spin(double s)
{
	// Reset message pointers before IPC exchange.
	globalpos_message = NULL;
	virtual_scan_message = NULL;
	carmen_ipc_sleep(s);
}

carmen_localize_ackerman_globalpos_message *carmen_get_globalpos()
{
	return globalpos_message;
}

carmen_virtual_scan_message *carmen_get_virtual_scan()
{
	return virtual_scan_message;
}

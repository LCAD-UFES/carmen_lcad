#include "carmen_gateway.h"

#include <iostream>

/** \brief Input global position message. */
static carmen_localize_ackerman_globalpos_message *globalpos_message;

/** @brief Offline map message. */
static carmen_mapper_map_message *offline_map_message;

/** \brief Input virtual scan message. */
static carmen_virtual_scan_message *virtual_scan_message;

/**
 * \brief Input virtual scan message hendler.
 */
static void carmen_localize_ackerman_globalpos_handler(void *msg)
{
	if (msg != NULL)
		globalpos_message = (carmen_localize_ackerman_globalpos_message*) msg;
}

/**
 * \brief Input virtual scan message hendler.
 */
static void carmen_offline_map_handler(void *msg)
{
	if (msg != NULL)
		offline_map_message = (carmen_mapper_map_message*) msg;
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
	// Reset message pointers.
	globalpos_message = NULL;
	offline_map_message = NULL;
	virtual_scan_message = NULL;

	carmen_ipc_initialize(argc, argv);

	carmen_localize_ackerman_subscribe_globalpos_message(
		NULL,
		carmen_localize_ackerman_globalpos_handler,
		CARMEN_SUBSCRIBE_LATEST
	);

	carmen_map_server_subscribe_offline_map(
		NULL,
		carmen_offline_map_handler,
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
	carmen_ipc_sleep(s);
}

carmen_localize_ackerman_globalpos_message *carmen_get_globalpos()
{
	return globalpos_message;
}

carmen_mapper_map_message *carmen_get_offline_map()
{
	return offline_map_message;
}

carmen_virtual_scan_message *carmen_get_virtual_scan()
{
	return virtual_scan_message;
}

#ifndef CARMEN_GATEWAY_H
#define CARMEN_GATEWAY_H

#include <carmen/carmen.h>
#include <carmen/localize_ackerman_interface.h>
#include <carmen/virtual_scan_interface.h>

/**
 * @brief Connect to the CARMEN network, setting up a node with the given initialization parameters.
 */
void carmen_gateway_init(int argc, char **argv);

/**
 * @brief Perform a communication cycle with the network, waiting the specified time for incoming messages.
 */
void carmen_gateway_spin(double s = 0.01);

/**
 * @brief Get the latest global position from the CARMEN network.
 *
 * @return Pointer to a global position message. May be `NULL`.
 */
carmen_localize_ackerman_globalpos_message *carmen_get_globalpos();

/**
 * @brief Get the latest virtual scan reading from the CARMEN network.
 *
 * @return Pointer to a virtual scan message. May be `NULL`.
 */
carmen_virtual_scan_message *carmen_get_virtual_scan();

#endif

#include "virtual_scan.h"

#include <carmen/carmen.h>
#include <carmen/virtual_scan_interface.h>

#include <iostream>

#include <cmath>

static carmen_virtual_scan_message *virtual_scan_message;

void carmen_virtual_scan_handler(void *msg)
{
	virtual_scan_message = (carmen_virtual_scan_message*) msg;
}

VirtualScan::VirtualScan(int argc, char **argv, g2d::Display *display, QObject *parent) : QObject(parent)
{
	this->display = display;

	carmen_ipc_initialize(argc, argv);

	carmen_virtual_scan_subscribe_message(
		NULL,
		carmen_virtual_scan_handler,
		CARMEN_SUBSCRIBE_LATEST
	);

	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(retrieve()));
	timer->start();
}

VirtualScan::~VirtualScan()
{
	carmen_ipc_disconnect();
}

void VirtualScan::retrieve()
{
	// Reset static message struct before checking IPC queue
	virtual_scan_message = NULL;
	carmen_ipc_sleep(0.01);
	if (virtual_scan_message == NULL)
		return;

	int n = virtual_scan_message->num_rays;
	g2d::Points &points = display->canvas.points;
	points.resize(n);

	double *t = virtual_scan_message->angles;
	double *d = virtual_scan_message->ranges;
	for (int i = 0; i < n; i++, t++, d++)
	{
		double ti = *t;
		double di = *d;

		double x = cos(ti) * di;
		double y = sin(ti) * di;
		points[i] = g2d::Point(x, y);
	}

	display->points.modelChanged();
}

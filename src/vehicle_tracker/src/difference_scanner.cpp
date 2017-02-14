#include "difference_scanner.h"

#include "carmen_gateway.h"

#include <iostream>

#include <cmath>

DifferenceScanner::DifferenceScanner(g2d::Display *display, QObject *parent) : QObject(parent)
{
	this->display = display;

	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start();
}

DifferenceScanner::~DifferenceScanner()
{
	carmen_ipc_disconnect();
}

void DifferenceScanner::update()
{
	carmen_gateway_spin();
	VirtualScan s2(carmen_get_globalpos(), carmen_get_virtual_scan());

	VirtualScan d(s1, s2);
	if (d.size() > 0)
		display->canvas.points = d.cartesian;

	if (s2.size() > 0)
		s1 = s2;

	display->points.modelChanged();
}

#include "vehicle_tracker_gui.h"

#include "carmen_gateway.h"

#include <QApplication>

int main(int argc, char *argv[])
{
	carmen_gateway_init(argc, argv);

	QApplication app(argc, argv);
	VehicleTrackerGUI w;
	w.show();

	return app.exec();
}

#include "vehicle_tracker_gui.h"

#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	VehicleTrackerGUI w(argc, argv);
	w.show();

	return app.exec();
}

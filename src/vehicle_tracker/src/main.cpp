#include "vehicle_tracker_gui.h"

#include <QApplication>

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);
	VehicleTrackerGUI w;
	w.show();

	return app.exec();
}

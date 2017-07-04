#include "vehicle_tracker_gui.h"
#include "ui_vehicle_tracker_gui.h"

#include "carmen_gateway.h"
#include "cluster2d.h"
#include "offline_map.h"
#include "virtual_scan.h"

#include <QApplication>

VehicleTrackerGUI::VehicleTrackerGUI(QWidget *parent):
	QMainWindow(parent),
	ui(new Ui::VehicleTrackerGUI),
	display_scan(&scene),
	display_map(&scene),
	display_clusters(&scene)
{
	ui->setupUi(this);

	scene.setItemIndexMethod(QGraphicsScene::NoIndex);
	ui->graphicsView->setScene(&scene);
	ui->graphicsView->scale(1, -1);

	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start();

	display_scan.circle(0, 0, 50);

	QPen pen_blue(::Qt::blue, 0, ::Qt::SolidLine);
	display_map.pen = pen_blue;
	display_map.points.setVerticesPen(pen_blue);

	QPen pen_green(::Qt::green, 0, ::Qt::SolidLine);
	display_clusters.pen = pen_green;
	display_clusters.points.setVerticesPen(pen_green);
}

VehicleTrackerGUI::~VehicleTrackerGUI()
{
	delete ui;
}

void VehicleTrackerGUI::resizeEvent(QResizeEvent * event)
{
	QMainWindow::resizeEvent(event);

	ui->graphicsView->fitInView(scene.sceneRect());
}

void VehicleTrackerGUI::update()
{
	carmen_gateway_spin();

	VirtualScan scan(carmen_get_globalpos(), carmen_get_virtual_scan());
	if (scan.size() > 0)
		display_scan.canvas.points = scan.cartesian;

	display_scan.points.modelChanged();

	display_map.clear();

	OfflineMap map(50, carmen_get_globalpos(), carmen_get_offline_map());
	if (map.size() > 0)
		display_map.canvas.points = map.cartesian.points;

	display_map.points.modelChanged();

	display_map.rectangle(0, 0, 2.0, 1.0, scan.angle);

	auto &clusters = map.cartesian.clusters;
	auto &points = display_clusters.canvas.points;

	points.clear();
	for (auto i = clusters.begin(), n = clusters.end(); i != n; ++i)
		points.insert(points.end(), i->begin(), i->end());

	display_clusters.points.modelChanged();

	//	display_clusters.clear();
// 	std::cout << map.cartesian.clusters.size() << std::endl;
// 	Clusters hulls = monotoneChain(map.cartesian.clusters);
// 	for (auto i = hulls.begin(), n = hulls.end(); i != n; ++i)
// 		display_clusters.polygon(*i);

// 	OfflineMap map(carmen_get_globalpos(), carmen_get_offline_map());
// 	if (map.size() > 0)
// 	{
// 		g2d::Field side = 0.5 * map.resolution;
// 		const std::vector<g2d::Point> &landmarks = map.cartesian;
// 		for (int i = 0, n = map.size(); i < n; i++)
// 		{
// 			const g2d::Point &p = landmarks[i];
// 			display.rectangle(p.x(), p.y(), side, side, 0);
// 		}
// 	}
}

int main(int argc, char *argv[])
{
	carmen_gateway_init(argc, argv);

	QApplication app(argc, argv);
	VehicleTrackerGUI w;
	w.show();

	return app.exec();
}

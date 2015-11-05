#include "main_window.h"
#include <QtGui>

#include "carmen/carmen_thread.h"

Main_Window::Main_Window(QWidget *parent)
: QMainWindow(parent)
{
	this->resize(600, 600);
	this->setWindowTitle("General Purpose GUI");

	//parte do mapa
	{
		view = new PrincipalView();
		view->scale(2, 2);
		setCentralWidget(view);

		QGraphicsScene *scene = new QGraphicsScene(QRect( 0, 0, 100, 100 ));
		view->setScene(scene);

		//item mapa
		{
			map_item = new Map_Item();
			map_item->setParent(view);
			scene->addItem(map_item);
			scene->addItem(map_item->temporary_car);
			map_item->setZValue(0.5);

			//connects
			connect(Carmen_Thread::getInstance(), SIGNAL(map_changed(carmen_map_t)),
					map_item, SLOT(set_map(carmen_map_t)));

			connect(Carmen_Thread::getInstance(), SIGNAL(occupancy_grid_changed(carmen_map_t)),
					map_item, SLOT(set_occupancy_grid_map(carmen_map_t)));
		}

		//localization and true pose
		{
			localization_pose = new Car_Component(Qt::green);
			scene->addItem(localization_pose);
			localization_pose->setZValue(20);

			true_pose = new Car_Component(Qt::blue);
			scene->addItem(true_pose);
			true_pose->setZValue(20);

			connect(Carmen_Thread::getInstance(), SIGNAL(truepos_changed(carmen_point_t)),
					true_pose,SLOT(set_pose(carmen_point_t)));

			connect(Carmen_Thread::getInstance(), SIGNAL(globalpos_changed(carmen_point_t)),
					localization_pose, SLOT(set_pose(carmen_point_t)));
		}

		//localize laser and normal laser
		{
			normal_laser = new Laser_Component(Qt::red);
			normal_laser->set_skip(20);
			scene->addItem(normal_laser);
			normal_laser->setZValue(1);

			localize_laser = new Laser_Component(Qt::yellow);
			localize_laser->set_skip(10);
			scene->addItem(localize_laser);
			localize_laser->setZValue(1);

			connect(Carmen_Thread::getInstance(), SIGNAL(truepos_changed(carmen_point_t)),
					normal_laser, SLOT(set_pose(carmen_point_t)));
			connect(Carmen_Thread::getInstance(), SIGNAL(globalpos_changed(carmen_point_t)),
					localize_laser, SLOT(set_pose(carmen_point_t)));

			connect(Carmen_Thread::getInstance(), SIGNAL(laser_changed(carmen_laser_laser_message)),
					normal_laser, SLOT(set_laser(carmen_laser_laser_message)));
			connect(Carmen_Thread::getInstance(), SIGNAL(localize_laser_changed(carmen_localize_ackerman_sensor_message)),
					localize_laser, SLOT(set_laser(carmen_localize_ackerman_sensor_message)));
		}
		//navigator waypoints
		{
			plan_component = new Navigator_Plan_Component();
			scene->addItem(plan_component);
			plan_component->setZValue(11);

			connect(Carmen_Thread::getInstance(), SIGNAL(plan_changed(carmen_navigator_ackerman_plan_message)),
					plan_component, SLOT(set_plan(carmen_navigator_ackerman_plan_message)));
		}
		//particles
		{
			particle = new Particle_Component();
			scene->addItem(particle);
			particle->setZValue(1.5);

			connect(Carmen_Thread::getInstance(), SIGNAL(particle_changed(carmen_localize_ackerman_particle_message)),
					particle, SLOT(set_particle(carmen_localize_ackerman_particle_message)));
		}
		//rrt component
		{
			rrt_robot_tree_component = new RRT_Component();
			scene->addItem(rrt_robot_tree_component);
			rrt_robot_tree_component->setZValue(10);

			connect(Carmen_Thread::getInstance(), SIGNAL(rrt_planner_robot_tree_changed(carmen_rrt_planner_tree_message)),
					rrt_robot_tree_component, SLOT(set_rrt(carmen_rrt_planner_tree_message)));


			rrt_goal_tree_component = new RRT_Component(Qt::green);
			scene->addItem(rrt_goal_tree_component);
			rrt_goal_tree_component->setZValue(10);

			connect(Carmen_Thread::getInstance(), SIGNAL(rrt_planner_goal_tree_changed(carmen_rrt_planner_tree_message)),
					rrt_goal_tree_component, SLOT(set_rrt(carmen_rrt_planner_tree_message)));
		}
		//goal component
		{
			goal_component = new Goal_Component();
			scene->addItem(goal_component);
			goal_component->setZValue(99);

			connect(Carmen_Thread::getInstance(), SIGNAL(navigator_status_changed(carmen_navigator_ackerman_status_message)),
					goal_component, SLOT(set_goal(carmen_navigator_ackerman_status_message)));

			connect(Carmen_Thread::getInstance(), SIGNAL(rrt_planner_status_changed(carmen_rrt_planner_status_message)),
					goal_component, SLOT(set_goal(carmen_rrt_planner_status_message)));

		}
	}

	add_toolbar();
	add_menu();
}

void Main_Window::add_toolbar() {
	QToolBar *toolbar;
	QAction *zoom_in, *zoom_out, *go, *stop;

	toolbar = new QToolBar("TOOLBAR", this);

	zoom_in = toolbar->addAction("Zoom In");
	zoom_in->setAutoRepeat(true);
	zoom_out = toolbar->addAction("Zoom Out");
	zoom_out->setAutoRepeat(true);

	go = toolbar->addAction("Go");
	stop = toolbar->addAction("Stop");

	connect(zoom_in, SIGNAL(triggered()),
			view, SLOT(zoomIn()));
	connect(zoom_out, SIGNAL(triggered()),
			view, SLOT(zoomOut()));
	connect(go, SIGNAL(triggered()),
			Carmen_Thread::getInstance(), SLOT(go()));
	connect(stop, SIGNAL(triggered()),
			Carmen_Thread::getInstance(), SLOT(stop()));

	this->addToolBar(toolbar);
}

void Main_Window::add_menu() {
	QMenuBar *menubar;

	menubar = new QMenuBar();

	//DISPLAY
	{
		QMenu* menu = menubar->addMenu("Display");
		{
			QMenu* sub_menu = menu->addMenu("Pose");
			{
				QAction* action = sub_menu->addAction("True Pose");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						true_pose, SLOT(set_visible(bool)));

				action->setChecked(true);
			}
			{
				QAction* action = sub_menu->addAction("Localization Pose");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						localization_pose, SLOT(set_visible(bool)));
				action->setChecked(true);
			}
			{
				QAction* action = sub_menu->addAction("Particles");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						particle, SLOT(set_visible(bool)));
				action->setChecked(true);
			}
			{
				QAction* action = sub_menu->addAction("Goal Pose");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						goal_component, SLOT(set_visible(bool)));
				action->setChecked(true);
			}
		}
		{
			QMenu* sub_menu = menu->addMenu("Laser");
			{
				QAction* action = sub_menu->addAction("Real Laser");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						normal_laser, SLOT(set_visible(bool)));
				action->setChecked(false);
			}
			{
				QAction* action = sub_menu->addAction("Localize Laser");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						localize_laser, SLOT(set_visible(bool)));
				action->setChecked(false);
			}
		}
		{
			QMenu* sub_menu = menu->addMenu("Navigator");
			{
				QAction* action = sub_menu->addAction("Plan");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						plan_component, SLOT(set_visible(bool)));
				action->setChecked(true);
			}
			{
				QAction* action = sub_menu->addAction("Waypoint");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						plan_component, SLOT(set_waypoint_visible(bool)));
				action->setChecked(true);
			}
			{
				QAction* action = sub_menu->addAction("RRT");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						rrt_robot_tree_component, SLOT(set_visible(bool)));

				connect(action, SIGNAL(toggled(bool)),
						rrt_goal_tree_component, SLOT(set_visible(bool)));

				action->setChecked(true);
			}
			{
				QAction* action = sub_menu->addAction("RRT Draw Car");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						rrt_robot_tree_component, SLOT(set_draw_car(bool)));

				connect(action, SIGNAL(toggled(bool)),
						rrt_goal_tree_component, SLOT(set_draw_car(bool)));

				action->setChecked(false);
			}
		}
		{
			QMenu* sub_menu = menu->addMenu("SLAM/Occupancy Grid");
			{
				QAction* action = sub_menu->addAction("Show");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						map_item, SLOT(set_occupancy_grid_visible(bool)));
				action->setChecked(false);
			}
			{
				QAction* action = sub_menu->addAction("Overlap");
				action->setCheckable(true);

				connect(action, SIGNAL(toggled(bool)),
						map_item, SLOT(set_occupancy_grid_overlap(bool)));
				action->setChecked(true);
			}
		}
	}

	this->setMenuBar(menubar);
}

void Main_Window::resizeEvent ( QResizeEvent * event ) {
	double new_scale = 0;


	view->scale((float)size().width()/size().width(), (float)event->size().width()/size().width());

	int x, y;

	x = Carmen_State::get_instance()->map_config->x_size;
	y = Carmen_State::get_instance()->map_config->y_size;

	if(!x || !y) {
		return;
	}

	view->resetTransform();

	new_scale = fmin((float)size().width()/x, (float)size().height()/y);

	view->scale(new_scale, new_scale);
}

Main_Window::~Main_Window()
{

}

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include "principal_view.h"
#include "map_item.h"
#include "map_component/laser_component.h"
#include "map_component/particle_component.h"
#include "map_component/car_component.h"
#include "map_component/navigator_plan_component.h"
#include "map_component/rrt_component.h"
#include "map_component/goal_component.h"

class Main_Window : public QMainWindow
{
	Q_OBJECT

public:
	Main_Window(QWidget *parent = 0);
	~Main_Window();

protected:
	void resizeEvent ( QResizeEvent * event );

public:
	PrincipalView *view;
	Map_Item* map_item;
	Car_Component *true_pose;
	Car_Component *localization_pose;
	Laser_Component* normal_laser;
	Laser_Component* localize_laser;
	Particle_Component* particle;
	Navigator_Plan_Component* plan_component;
	RRT_Component *rrt_robot_tree_component;
	RRT_Component *rrt_goal_tree_component;
	Goal_Component *goal_component;

private:
void add_toolbar();
void add_menu();

};

#endif // MAIN_WINDOW_H

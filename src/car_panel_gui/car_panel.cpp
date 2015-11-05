/*
 * car_panel.cpp
 *
 *  Created on: Apr 19, 2013
 *      Author: cayo
 */

#include "car_panel.h"


car_panel *car_panel::instance = NULL;


car_panel::car_panel(int argc, char *argv[])
{
	if (!checkArguments(argc, argv))
	{
		subscribe_messages(2, 1.0);
	}
}


car_panel* 
car_panel::get_instance(int argc, char *argv[])
{
    if (instance == NULL)
    {
        instance = new car_panel(argc, argv);
    }

    return instance;
}


void
car_panel::set_type_message(int type_message)
{
	setTypeMessage(type_message);
}


void
car_panel::set_view(int w, int h)
{
	reshape(w, h);
}


void
car_panel::draw(void)
{
	display();
}

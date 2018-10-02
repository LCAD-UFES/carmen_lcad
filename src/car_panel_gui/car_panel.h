/*
 * car_panel.h
 *
 *  Created on: Apr 19, 2013
 *      Author: cayo
 */

#ifndef CAR_PANEL_H_
#define CAR_PANEL_H_

#include <stddef.h>

class car_panel
{
	public:
		void set_type_message(int);
		void set_turn_signal(int);
		void set_view(int, int);
		void draw(void);

		static car_panel *get_instance(int argc, char *argv[]);	

	private:
		car_panel(int argc, char *argv[]);

		static car_panel *instance;
};

#endif /* CAR_PANEL_H_ */

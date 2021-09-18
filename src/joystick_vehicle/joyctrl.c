/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/global.h>
#include "joyctrl.h"

int carmen_initialize_joystick(carmen_joystick_type *joystick)
{
	if ((joystick->fd = open(CARMEN_JOYSTICK_DEVICE, O_RDONLY | O_NONBLOCK)) < 0)
	{
		carmen_warn("Warning: could not initialize joystick.\n");
		joystick->initialized = 0;
		return -1;
	}
	ioctl(joystick->fd, JSIOCGAXES, &joystick->nb_axes);
	ioctl(joystick->fd, JSIOCGBUTTONS, &joystick->nb_buttons);

	char name[128];
	if (ioctl(joystick->fd, JSIOCGNAME(sizeof(name)), name) < 0)
		strncpy(name, "Unknown", sizeof(name));
	printf("Name: %s\n", name);



	if (strstr(name,"RumblePad 2")!=NULL)
	{
		joystick->type = RUMBLEPAD2;
		printf("Detected Rumblepad 2\n");
	} 

	else if (strstr(name,"Wingman")!=NULL)
	{
		joystick->type = WINGMAN;
		printf("Detected Wingman.\n");
	}

	joystick->axes = (int *)calloc(joystick->nb_axes, sizeof(int));
	carmen_test_alloc(joystick->axes);
	joystick->buttons = (int *)calloc(joystick->nb_buttons, sizeof(int));
	carmen_test_alloc(joystick->buttons);
	joystick->initialized = 1;

	return 0;
}

void carmen_set_deadspot(carmen_joystick_type *joystick, 
		int on_off, double size)
{
	joystick->deadspot = on_off;
	joystick->deadspot_size = size;
}

int carmen_get_joystick_state(carmen_joystick_type *joystick)
{
	struct js_event mybuffer[64];
	int n, i;

	if (joystick->initialized == 0)
		return -1;

	n = read (joystick->fd, mybuffer, sizeof(struct js_event) * 64);

	if (n != -1)
	{
		for(i = 0; i < n / (signed int)sizeof(struct js_event); i++)
		{
			if(mybuffer[i].type & JS_EVENT_BUTTON &~ JS_EVENT_INIT)
			{
				joystick->buttons[mybuffer[i].number] = mybuffer[i].value;
			}
			else if(mybuffer[i].type == JS_EVENT_AXIS)
			{
				joystick->axes[mybuffer[i].number] = mybuffer[i].value;

				if(mybuffer[i].number == 0 || mybuffer[i].number == 1)
				{
					if(joystick->deadspot)
					{
						if(abs(joystick->axes[mybuffer[i].number]) < joystick->deadspot_size * 32767)
							joystick->axes[mybuffer[i].number] = 0;
						else if(joystick->axes[mybuffer[i].number] > 0)
							joystick->axes[mybuffer[i].number] = (joystick->axes[mybuffer[i].number] -
											joystick->deadspot_size * 32767) / ((1-joystick->deadspot_size) * 32767) * 32767.0;
						else if(joystick->axes[mybuffer[i].number] < 0)
							joystick->axes[mybuffer[i].number] =
									(joystick->axes[mybuffer[i].number] +
											joystick->deadspot_size * 32767) / ((1-joystick->deadspot_size) * 32767) * 32767.0;
					} else
						joystick->axes[mybuffer[i].number] = joystick->axes[mybuffer[i].number];
				}
				if(mybuffer[i].number == 1 || mybuffer[i].number == 5)
					joystick->axes[mybuffer[i].number] *= -1;
			}
		}
	}
	return n;
}

void carmen_close_joystick(carmen_joystick_type *joystick)
{
	if (joystick->initialized == 0)
		return;

	close(joystick->fd);
	free(joystick->axes);
	free(joystick->buttons);
}

void carmen_joystick_control(carmen_joystick_type *joystick, double max_right_analogic_button_value, 
		double max_left_analogic_button_value, double *right_analogic_button_value, double *left_analogic_button_value)
{
	if (joystick->initialized == 0)
		return;

	//printfs just for debug propose
	/*int i;
	printf("\nJOYSTICK AXES\n");
	for(i=0; i<joystick->nb_axes;i++) {
		printf("axes[%d] %d\n", i, joystick->axes[i]);
	}

	printf("\nJOYSTICK Buttons\n");
	for(i=0; i<joystick->nb_buttons;i++) {
		printf("buttons[%d] %d\n", i, joystick->buttons[i]);
	}

	printf("deadspot %d\n", joystick->deadspot);
	printf("deadspot_size %f\n", joystick->deadspot_size);
	printf("fd %d\n", joystick->fd);
	printf("initialized %d\n", joystick->initialized);
	printf("type %d\n", joystick->type);*/

	*right_analogic_button_value = -1* (joystick->axes[4] / 32767.0) * max_right_analogic_button_value;

	//X
	if(joystick->buttons[2]) {
		*right_analogic_button_value = max_right_analogic_button_value;
	}

	//quadrado
	if(joystick->buttons[3]) {
		*right_analogic_button_value = -max_right_analogic_button_value;
	}

	*left_analogic_button_value = -1 * (joystick->axes[0] / 32767.0) * max_left_analogic_button_value;
	carmen_verbose("axes[0]: %d axes[1]: %d tv: %f rv: %f\n",
			joystick->axes[0], joystick->axes[0], *right_analogic_button_value, *left_analogic_button_value);
}

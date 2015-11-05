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

#ifdef __cplusplus
extern "C" {
#endif

/* Initialization of asynchronous keyboard; *
 * has to be called first                   */
void carmen_initialize_keyboard(void);

/* Read most recent character from keybord; *
 * returns 0 if no character could be read, *
 * 1 otherwise.                             */
int carmen_read_char(char *c);

/* Robot control via the keyboard; input is *
 * a character, output are translational and*
 * rotational velocities. The function will *
 * return -1 if 'quit' was selected, else 0 */
int carmen_keyboard_control(char c, double max_tv, double max_rv, 
			    double *tv, double *rv);

#ifdef __cplusplus
}
#endif

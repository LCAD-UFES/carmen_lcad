/*
 * panel_interface.h
 *
 *  Created on: May 6, 2013
 *      Author: cayo
 */

#ifndef PANEL_INTERFACE_H_
#define PANEL_INTERFACE_H_
 
int checkArguments(int argc, char *argv[]);

void display(void);

void reshape(int w, int h);

void subscribe_messages(int, double);

void setTypeMessage(int);

#endif /* PANEL_INTERFACE_H_ */

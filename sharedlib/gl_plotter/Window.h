#ifndef WINDOW_H_
#define WINDOW_H_

#include <stdlib.h>
#include <stdio.h>
#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

typedef struct window window;

window* initWindow(char* window_name, int width, int height);

void makeWindowCurrent(window* w);

int showWindow(window* w);

int processWindow(window* w, void (*mouseFunc)(int type, int button, int x, int y), void (*keyPress)(int code), void (*keyRelease)(int code));

void destroyWindow(window* w);

#endif

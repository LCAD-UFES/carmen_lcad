#ifndef __KINECT_GLVIEW_H
#define __KINECT_GLVIEW_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ON_DRAW_SCENE)(void);
typedef void (*ON_KEY_PRESSED)(unsigned char key, int x, int y);

void setDrawSceneHandler(ON_DRAW_SCENE handler);
void setKeyPressedHandler(ON_KEY_PRESSED handler);

void VideoCallback(int kinect_id, uint8_t* _video, uint32_t timestamp, int size) ;

void DepthCallback(int kinect_id, uint16_t* _depth, uint32_t timestamp, int size) ;

void display(int *argc, char **argv);

void keyPressed(unsigned char key, int x, int y);

#ifdef __cplusplus
}
#endif

#endif

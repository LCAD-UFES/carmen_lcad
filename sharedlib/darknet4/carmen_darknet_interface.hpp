#ifndef __DARKNET_INTERFACE_H__
#define __DARKNET_INTERFACE_H__

#include <vector>
#include "bbox.h"

char **
get_classes_names(char *classes_names_file);

void*
load_yolo_network(char *cfg, char *weights, int clear);

std::vector<bbox_t>
run_YOLO(unsigned char *data, int w, int h, float threshold);

#endif

#pragma once
#define STRIP_FLAG_HELP 1  
#include "gflags/gflags.h"

DECLARE_string(labels_list_file);
DECLARE_string(config_file);
DECLARE_string(weights_file);
DECLARE_string(images_list_file);
DECLARE_string(running_type);
DECLARE_string(output_acc_dir);
#!/bin/bash

cd map_server_multi_height
make
cd ../mapper_multi_height2
make
cd ../osbtacle_avoider_multi_height
make
cd ../obstacle_distance_mapper_multi_height
make
cd ../behavior_selector_multi_height
make
cd ../model_predictive_planner_multi_height
make
cd ..

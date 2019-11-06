#!/bin/bash

cd map_server_multi_height
make clean all
cd ../mapper_multi_height2
make clean all
cd ../obstacle_distance_mapper_multi_height
make clean all
cd ../obstacle_avoider_multi_height
make clean all
cd ../behavior_selector_multi_height
make clean all
cd ../model_predictive_planner_multi_height
make clean all
cd ..

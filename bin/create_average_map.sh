#!/bin/bash

./build_complete_map -map_path ../data/mapper_teste2/ -map_resolution 0.2 -map_type u
./build_complete_map -map_path ../data/mapper_teste2/ -map_resolution 0.2 -map_type o
./build_complete_map -map_path ../data/mapper_teste2/ -map_resolution 0.2 -map_type e
sleep 4
./build_average_map -map_path ../data/mapper_teste2/
sleep 4
cp ../data/mapper_teste2/complete_map_count.info ../data/mapper_teste2/complete_map.info
sleep 1
./complete_map_to_bitmap -map_path ../data/mapper_teste2/
sleep 4
eog ../data/mapper_teste2/complete_bitmap.bmp &
echo "ok"


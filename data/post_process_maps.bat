#!/bin/bash
FILES=`ls split_map_*`
for file in $FILES; do
	echo $CARMEN_HOME/bin/complete_map_to_block_map -map_path dir_${file} -map_resolution 0.2 -block_size_in_meters 210.0
	$CARMEN_HOME/bin/complete_map_to_block_map -map_path dir_${file} -map_resolution 0.2 -block_size_in_meters 210.0
	MAPS=`cat ${file}`
	for map in $MAPS; do
		## separates the name of the map from the name of its original directory
		## DIR_NAME=`echo $map | tr "/" "\n" | head -n -1 | tr "\n" "/"`
		MAP_NAME=`echo $map | tr "/" "\n" | tail -1`
		# perform the copy
		echo cp dir_${file}/$MAP_NAME $map
		cp dir_${file}/$MAP_NAME $map
	done
done

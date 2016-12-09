#!/bin/bash
FILES=`ls split_map*`
for file in $FILES; do
	mkdir dir_${file}
	MAPS=`cat ${file}`
	for map in $MAPS; do
		cp $map dir_${file}/
	done
	echo $CARMEN_HOME/bin/build_complete_map -map_path dir_${file} -map_resolution 0.2 -map_type m
	$CARMEN_HOME/bin/build_complete_map -map_path dir_${file} -map_resolution 0.2 -map_type m
done

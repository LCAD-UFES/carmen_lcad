[[ ! -f i$1_-$2.svg ]] && echo Usage: $0 "<x_origin> <unsigned_y_origin>" && echo File not found: i$1_-$2.svg && exit 
[[ -f r$1_-$2.map ]] && rm r$1_-$2.map
python $CARMEN_HOME/src/road_mapper/road_mapper_generate_gt4.py  -o . -n -l 7 i$1_-$2.svg
$CARMEN_HOME/src/road_mapper/road_mapper_display_map3 r$1_-$2.map

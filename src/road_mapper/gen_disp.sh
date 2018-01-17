usage="<x_origin> <unsigned_y_origin> [<left_distance_units>]"
[[ ! -f i$1_-$2.svg ]] && echo Usage: $0 $usage && echo File not found: i$1_-$2.svg && exit 
left_distance=7.0
re='^[0-9]+([.][0-9]+)?$'
if [[ $# > 2 ]] ; then
	! [[ $3 =~ $re ]] && echo Usage: $0 $usage && echo "Left distance is not a valid positive floating point number:" $3 && exit
	left_distance=$3 
fi
[[ -f r$1_-$2.map ]] && rm r$1_-$2.map
python $CARMEN_HOME/src/road_mapper/road_mapper_generate_gt4.py  -n -x -o . -l $left_distance i$1_-$2.svg
$CARMEN_HOME/src/road_mapper/road_mapper_display_map r$1_-$2.map

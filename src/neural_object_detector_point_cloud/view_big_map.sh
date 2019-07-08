#!/bin/bash

USAGE="
Usage: $0 <directory> <file_pattern> <map_width_meters> <map_height_meters>
  file_pattern:       default = m{x}_{y}.map.png
  map_width_meters:   default = 70
  map_height_meters:  default = <map_width_meters>
"

######## Prerequisites ##########

graphics_software="inkscape"
software_source="http://www.inkscape.org"

if [[ ! -x $(command -v "$graphics_software") ]]; then 
	echo -e "\nThis script requires the $graphics_software graphics software. You may download and install it from $software_source\n"
	exit
fi

########### Defaults ############

file_pattern="m{x}_{y}.map.png"
map_width=70

########## Functions ############

function usage()
{
	echo -e "$USAGE"
	exit $1
}

function usage_error()
{
	echo -e "\n$1"
	usage 1
}

function is_file_pattern_ok()
{
	if [[ $1 != *{x}* ]] || [[ $1 != *{y}* ]] || [[ $1 = *{x}*{x}* ]] || [[ $1 = *{y}*{y}* ]]; then
		echo -e "\nFile pattern must have exactly one {x} and one {y}"
		return 1
	fi
	x_mask=${file_pattern/'{y}'/*}
	x_prefix=${x_mask%'{x}'*}
	x_suffix=${x_mask#*'{x}'}
	y_mask=${file_pattern/'{x}'/*}
	y_prefix=${y_mask%'{y}'*}
	y_suffix=${y_mask#*'{y}'}
	file_wildcard=${y_mask/'{y}'/*}
}

function get_x_from_filename()
{
	x_value=${1#$x_prefix}
	x_value=${x_value%$x_suffix}
	echo $x_value
}

function get_y_from_filename()
{
	y_value=${1#$y_prefix}
	y_value=${y_value%$y_suffix}
	echo $y_value
}

function get_imagetype()
{
	valid_types="PNG JPEG"
	for t in $valid_types; do
		tag="*$t image data*"
		if [[ $1 = $tag ]]; then
			echo $t
			return
		fi
	done
	echo 'invalid'
}

function get_imagesize()
{
	size=0x0
	imagetype=$(get_imagetype "$1")
	case $imagetype in

		PNG) 
			size=${1#*"image data, "}
			size=${size%%", "*}
			size=${size// /}
			;;
		JPEG)
			size=${1#*"precision"*", "}
			size=${size%%", "*}
			;;
	esac
	echo $size
}

function float_min()
{
	evaluation=$(bc <<< "$1 < $2" 2>/dev/null)
	if [[ $evaluation = 1 ]]; then
		echo "$1"
	else
		echo "$2"
	fi
}

function float_test()
{
	evaluation=$(bc <<< "$1" 2>/dev/null)
	test $evaluation = 1
	return $?
}

function is_float()
{
	[[ $1 =~ ^[-]?[0-9]+\.?[0-9]*$ ]] || [[ $1 =~ ^[-]?[0-9]*\.?[0-9]+$ ]]
	return $? 
}

function is_positive_float()
{
	if ! is_float "$1"; then return 1; fi
	float_test "$1 > 0"
	return $? 
}

########### Parameters ############

if (( $# == 0 )); then usage; fi
if (( $# >  4 )); then usage_error "Too many arguments"; fi
if [[ ! -d $1 ]]; then usage_error "$1: No such directory"; else directory=$1; fi
if (( $# >= 2 )); then file_pattern=$2; fi
if ! is_file_pattern_ok $file_pattern; then usage_error "$2: Invalid file pattern"; fi
if (( $# >= 3 )); then map_width=$3; fi
if ! is_positive_float $map_width;  then usage_error "$3: Invalid map width in meters";  fi
if (( $# >= 4 )); then map_height=$4; else map_height=$map_width; fi
if ! is_positive_float $map_height; then usage_error "$4: Invalid map height in meters"; fi

###################################

count=0
for filename in "$directory"/$file_wildcard; do
	fileinfo=$(file "$filename")
	fileinfo=${fileinfo#"$filename: "}
	imagesize=$(get_imagesize "$fileinfo")
	href=$(basename "$filename")
	if [[ $imagesize = 0x0 ]]; then
		echo -e "\n$href: Unable to use this file type: $fileinfo"
		continue
	fi
	x=$(get_x_from_filename "$href")
	y=$(get_y_from_filename "$href")
	if ! is_float "$x" || ! is_float "$y"; then
		echo -e "\n$href: Invalid map coordinates (x,y): ($x,$y)"
		continue
	fi
	if (( $count == 0 )); then
		common_imagesize=$imagesize
		width=${imagesize%x*}
		height=${imagesize#*x}
		x_min=$x; x_max=$x
		y_min=$y; y_max=$y
	fi
	if [[ $imagesize != $common_imagesize ]]; then
		echo -e "\n$href: Image size does not match the other files: $fileinfo"
		continue
	fi
	
	let count++
	if (( $x_min > $x )); then x_min=$x; fi
	if (( $x_max < $x )); then x_max=$x; fi
	if (( $y_min > $y )); then y_min=$y; fi
	if (( $y_max < $y )); then y_max=$y; fi
done
echo -e "\n$count $file_pattern files read\n"

total_width=$( bc <<< "( ( $x_max - $x_min ) / $map_width  + 1 ) * $width")
total_height=$(bc <<< "( ( $y_max - $y_min ) / $map_height + 1 ) * $height")
zoom_width=$( bc <<< "scale=4; 1430 * 0.9 / $total_width")
zoom_height=$(bc <<< "scale=4;  900 * 0.9 / $total_height")
zoom=$(float_min "$zoom_width" "$zoom_height")
cx=$(bc <<< "( $total_width  / 2 ) + ( 154 / $zoom )")
cy=$(bc <<< "( $total_height / 2 )")
docname=complete_"$x_min"_"$y_min".map.svg
graphics_file="$directory/$docname"

content=
content+='<?xml version="1.0" encoding="UTF-8" standalone="no"?>\n'
content+='<!-- Created with Inkscape (http://www.inkscape.org/) -->\n'
content+='<svg\n'
content+='   xmlns:dc="http://purl.org/dc/elements/1.1/"\n'
content+='   xmlns:cc="http://creativecommons.org/ns#"\n'
content+='   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"\n'
content+='   xmlns:svg="http://www.w3.org/2000/svg"\n'
content+='   xmlns="http://www.w3.org/2000/svg"\n'
content+='   xmlns:xlink="http://www.w3.org/1999/xlink"\n'
content+='   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"\n'
content+='   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"\n'
content+='   width="'$total_width'"\n'
content+='   height="'$total_height'"\n'
content+='   viewBox="0 0 '$total_width' '$total_height'"\n'
content+='   version="1.1"\n'
content+='   id="svg8"\n'
content+='   inkscape:version="0.92.2 (unknown)"\n'
content+='   sodipodi:docname="'$docname'">\n'
content+='  <defs\n'
content+='     id="defs2" />\n'
content+='  <sodipodi:namedview\n'
content+='     id="base"\n'
content+='     pagecolor="#ffffff"\n'
content+='     bordercolor="#666666"\n'
content+='     borderopacity="1.0"\n'
content+='     inkscape:pageopacity="0.0"\n'
content+='     inkscape:pageshadow="2"\n'
content+='     inkscape:zoom="'$zoom'"\n'
content+='     inkscape:cx="'$cx'"\n'
content+='     inkscape:cy="'$cy'"\n'
content+='     inkscape:document-units="px"\n'
content+='     inkscape:current-layer="layer1"\n'
content+='     showgrid="false"\n'
content+='     units="px"\n'
content+='     inkscape:window-width="1855"\n'
content+='     inkscape:window-height="1056"\n'
content+='     inkscape:window-x="65"\n'
content+='     inkscape:window-y="24"\n'
content+='     inkscape:window-maximized="1" />\n'
content+='  <metadata\n'
content+='     id="metadata5">\n'
content+='    <rdf:RDF>\n'
content+='      <cc:Work\n'
content+='         rdf:about="">\n'
content+='        <dc:format>image/svg+xml</dc:format>\n'
content+='        <dc:type\n'
content+='           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />\n'
content+='        <dc:title></dc:title>\n'
content+='      </cc:Work>\n'
content+='    </rdf:RDF>\n'
content+='  </metadata>\n'
content+='  <g\n'
content+='     inkscape:label="Layer 1"\n'
content+='     inkscape:groupmode="layer"\n'
content+='     id="layer1"\n'
content+='     transform="translate(0,0)">\n'

for filename in "$directory"/$file_wildcard; do
	fileinfo=$(file "$filename")
	fileinfo=${fileinfo#"$filename: "}
	imagesize=$(get_imagesize "$fileinfo")
	href=$(basename "$filename")
	absref=$(cd $(dirname "$filename"); pwd)/"$href"
	x=$(get_x_from_filename "$href")
	y=$(get_y_from_filename "$href")
	if ! is_float "$x" || ! is_float "$y"; then continue; fi
	if [[ $imagesize != $common_imagesize ]]; then continue; fi

	x_img=$(bc <<< "( ( $x - $x_min ) / $map_width ) * $width")
	y_img=$(bc <<< "$total_height - ( ( ( ( $y - $y_min ) / $map_height ) + 1 ) * $height )")

	content+='    <image\n'
	content+='       sodipodi:absref="'$absref'"\n'
	content+='       xlink:href="'$href'"\n'
	content+='       id="'$href'"\n'
	content+='       x="'$x_img'"\n'
	content+='       y="'$y_img'"\n'
	content+='       width="'$width'"\n'
	content+='       height="'$height'"\n'
	content+='       preserveAspectRatio="none" />\n'
done

content+='  </g>\n'
content+='</svg>'
echo -e "$content" > $graphics_file
set -x
$graphics_software $graphics_file 2> /dev/null &


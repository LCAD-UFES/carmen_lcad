#!/bin/bash
USAGE="Usage: $0 <map_png_directory> [<map_length_meters>]"
nl='\n'
if ! [ -x "$(command -v inkscape)" ]; then echo "This script requires the Inkscape graphics software. You may download and install it from http://www.inkscape.org"; exit; fi
if [ $# -eq 0 ]; then echo "$USAGE"; exit; fi
if [ $# -gt 2 ]; then echo -e "Too many arguments"$nl"$USAGE"; exit; fi
directory=$1
if ! [ -d $directory ]; then echo -e "$1: No such directory"$nl"$USAGE"; exit; fi
if [ $# -eq 2 ]; then map_length=$(bc <<< $2 2> /dev/null); else map_length=70; fi
if [ "$map_length" = "" ] || [ $(bc <<< "$map_length <= 0") -eq 1 ]; then echo -e "$2: Invalid map length in meters"$nl"$USAGE"; exit; fi

count=0
for filename in $directory/m*_*.map.png; do
	fileinfo=$(file $filename)
	fileinfo=${fileinfo#"$filename: "}

	x=${filename#$directory/m}
	x=${x%_*.map.png}
	y=${filename#$directory/m*_}
	y=${y%.map.png}

	if [ $count -eq 0 ]; then
		common_fileinfo=$fileinfo
		width=${fileinfo#"PNG image data, "}
		width=${width%%" x "*}
		height=${fileinfo#"PNG image data, "*" x "}
		height=${height%%", "*}

		x_min=$x
		x_max=$x
		y_min=$y
		y_max=$y
	fi

	if [ "$fileinfo" != "$common_fileinfo" ]; then
		echo "$(basename $filename): File disregarded because file info does not match the other files: $fileinfo"
		continue
	fi

	let "count++"
	if [ $x_min -gt $x ]; then x_min=$x; fi
	if [ $x_max -lt $x ]; then x_max=$x; fi
	if [ $y_min -gt $y ]; then y_min=$y; fi
	if [ $y_max -lt $y ]; then y_max=$y; fi
done

echo "$count map PNG files read"
let "total_width  = ( ( x_max - x_min ) / map_length + 1 ) * width"
let "total_height = ( ( y_max - y_min ) / map_length + 1 ) * height"
zoom_width=$(bc <<< "scale=4; 1430 * 0.9 / $total_width")
zoom_height=$(bc <<< "scale=4; 900 * 0.9 / $total_height")
if [ $(bc <<< "$zoom_width <  $zoom_height") -eq 1 ]; then zoom=$zoom_width; else zoom=$zoom_height; fi
cx=$(bc <<< "( $total_width  / 2 )  + ( 154 / $zoom )")
cy=$(bc <<< "( $total_height / 2 )")
docname=complete_"$x_min"_"$y_min".map.svg
inkscape_file=$directory/$docname

content=
content+='<?xml version="1.0" encoding="UTF-8" standalone="no"?>'$nl
content+='<!-- Created with Inkscape (http://www.inkscape.org/) -->'$nl
content+='<svg'$nl
content+='   xmlns:dc="http://purl.org/dc/elements/1.1/"'$nl
content+='   xmlns:cc="http://creativecommons.org/ns#"'$nl
content+='   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"'$nl
content+='   xmlns:svg="http://www.w3.org/2000/svg"'$nl
content+='   xmlns="http://www.w3.org/2000/svg"'$nl
content+='   xmlns:xlink="http://www.w3.org/1999/xlink"'$nl
content+='   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"'$nl
content+='   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"'$nl
content+='   width="'$total_width'"'$nl
content+='   height="'$total_height'"'$nl
content+='   viewBox="0 0 '$total_width' '$total_height'"'$nl
content+='   version="1.1"'$nl
content+='   id="svg8"'$nl
content+='   inkscape:version="0.92.2 (unknown)"'$nl
content+='   sodipodi:docname="'$docname'">'$nl
content+='  <defs'$nl
content+='     id="defs2" />'$nl
content+='  <sodipodi:namedview'$nl
content+='     id="base"'$nl
content+='     pagecolor="#ffffff"'$nl
content+='     bordercolor="#666666"'$nl
content+='     borderopacity="1.0"'$nl
content+='     inkscape:pageopacity="0.0"'$nl
content+='     inkscape:pageshadow="2"'$nl
content+='     inkscape:zoom="'$zoom'"'$nl
content+='     inkscape:cx="'$cx'"'$nl
content+='     inkscape:cy="'$cy'"'$nl
content+='     inkscape:document-units="px"'$nl
content+='     inkscape:current-layer="layer1"'$nl
content+='     showgrid="false"'$nl
content+='     units="px"'$nl
content+='     inkscape:window-width="1855"'$nl
content+='     inkscape:window-height="1056"'$nl
content+='     inkscape:window-x="65"'$nl
content+='     inkscape:window-y="24"'$nl
content+='     inkscape:window-maximized="1" />'$nl
content+='  <metadata'$nl
content+='     id="metadata5">'$nl
content+='    <rdf:RDF>'$nl
content+='      <cc:Work'$nl
content+='         rdf:about="">'$nl
content+='        <dc:format>image/svg+xml</dc:format>'$nl
content+='        <dc:type'$nl
content+='           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />'$nl
content+='        <dc:title></dc:title>'$nl
content+='      </cc:Work>'$nl
content+='    </rdf:RDF>'$nl
content+='  </metadata>'$nl
content+='  <g'$nl
content+='     inkscape:label="Layer 1"'$nl
content+='     inkscape:groupmode="layer"'$nl
content+='     id="layer1"'$nl
content+='     transform="translate(0,0)">'$nl

for filename in $directory/m*_*.map.png; do
	fileinfo=$(file $filename)
	fileinfo=${fileinfo#"$filename: "}
	if [ "$fileinfo" != "$common_fileinfo" ]; then continue; fi

	href=$(basename $filename)
	absref=$(cd $(dirname $filename); pwd)/$href

	x=${filename#$directory/m}
	x=${x%_*.map.png}
	y=${filename#$directory/m*_}
	y=${y%.map.png}

	let "x_img = ( ( x - x_min ) / map_length ) * width"
	let "y_img = total_height - ( ( ( ( y - y_min ) / map_length ) + 1 ) * height )"

	content+='    <image'$nl
	content+='       sodipodi:absref="'$absref'"'$nl
	content+='       xlink:href="'$href'"'$nl
	content+='       id="'$href'"'$nl
	content+='       x="'$x_img'"'$nl
	content+='       y="'$y_img'"'$nl
	content+='       width="'$width'"'$nl
	content+='       height="'$height'"'$nl
	content+='       preserveAspectRatio="none" />'$nl
done

content+='  </g>'$nl
content+='</svg>'
echo -e "$content" > $inkscape_file

set -x
inkscape $inkscape_file 2> /dev/null &


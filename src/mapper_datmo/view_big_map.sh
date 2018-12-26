#!/bin/bash
! [ -f /usr/bin/inkscape ] && echo "This script requires the Inkscape graphics software. Please download it from http://inkscape.org" && exit
USAGE="Usage: $0 <map_png_directory> [<map_length_meters>]"
[ $# -gt 2 ] && echo "$USAGE" && exit
directory=$1
! [ -d $directory ] && echo "$1: No such directory" && echo "$USAGE" && exit
map_length=70
[ $# -eq 2 ] && map_length=`echo $2 | bc 2> /dev/null`
[ "$map_length" = "" ] && echo "$2: Invalid map length in meters" && echo "$USAGE" && exit

count=0
for filename in $directory/m*_*.map.png
do
	fileinfo=`file $filename`
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

	[ "$fileinfo" != "$common_fileinfo" ] && echo "$filename: File disregarded because file info does not match the other files: $fileinfo" && continue

	let "count++"
	[ $x_min -gt $x ] && x_min=$x
	[ $x_max -lt $x ] && x_max=$x
	[ $y_min -gt $y ] && y_min=$y
	[ $y_max -lt $y ] && y_max=$y
done

let "total_width  = ( ( x_max - x_min ) / map_length + 1 ) * width"
let "total_height = ( ( y_max - y_min ) / map_length + 1 ) * height"
let "cx = total_width / 2"
let "cy = total_height / 2"
zoom_width=` echo "scale=4; 1430 / $total_width " | bc`
zoom_height=`echo "scale=4;  900 / $total_height" | bc`
[ `echo "$zoom_width <  $zoom_height" | bc` -eq 1 ] && zoom=$zoom_width || zoom=$zoom_height
docname=complete_"$x_min"_"$y_min".map.svg
inkscape_file=$directory/$docname

echo '<?xml version="1.0" encoding="UTF-8" standalone="no"?>' > $inkscape_file
echo '<!-- Created with Inkscape (http://www.inkscape.org/) -->' >> $inkscape_file
echo '<svg' >> $inkscape_file
echo '   xmlns:dc="http://purl.org/dc/elements/1.1/"' >> $inkscape_file
echo '   xmlns:cc="http://creativecommons.org/ns#"' >> $inkscape_file
echo '   xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#"' >> $inkscape_file
echo '   xmlns:svg="http://www.w3.org/2000/svg"' >> $inkscape_file
echo '   xmlns="http://www.w3.org/2000/svg"' >> $inkscape_file
echo '   xmlns:xlink="http://www.w3.org/1999/xlink"' >> $inkscape_file
echo '   xmlns:sodipodi="http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"' >> $inkscape_file
echo '   xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape"' >> $inkscape_file
echo '   width="'$total_width'"' >> $inkscape_file
echo '   height="'$total_height'"' >> $inkscape_file
echo '   viewBox="0 0 '$total_width' '$total_height'"' >> $inkscape_file
echo '   version="1.1"' >> $inkscape_file
echo '   id="svg8"' >> $inkscape_file
echo '   inkscape:version="0.92.2 (unknown)"' >> $inkscape_file
echo '   sodipodi:docname="'$docname'">' >> $inkscape_file
echo '  <defs' >> $inkscape_file
echo '     id="defs2" />' >> $inkscape_file
echo '  <sodipodi:namedview' >> $inkscape_file
echo '     id="base"' >> $inkscape_file
echo '     pagecolor="#ffffff"' >> $inkscape_file
echo '     bordercolor="#666666"' >> $inkscape_file
echo '     borderopacity="1.0"' >> $inkscape_file
echo '     inkscape:pageopacity="0.0"' >> $inkscape_file
echo '     inkscape:pageshadow="2"' >> $inkscape_file
echo '     inkscape:zoom="'$zoom'"' >> $inkscape_file
echo '     inkscape:cx="'$cx'"' >> $inkscape_file
echo '     inkscape:cy="'$cy'"' >> $inkscape_file
echo '     inkscape:document-units="px"' >> $inkscape_file
echo '     inkscape:current-layer="layer1"' >> $inkscape_file
echo '     showgrid="false"' >> $inkscape_file
echo '     units="px"' >> $inkscape_file
echo '     inkscape:window-width="1855"' >> $inkscape_file
echo '     inkscape:window-height="1056"' >> $inkscape_file
echo '     inkscape:window-x="65"' >> $inkscape_file
echo '     inkscape:window-y="24"' >> $inkscape_file
echo '     inkscape:window-maximized="1" />' >> $inkscape_file
echo '  <metadata' >> $inkscape_file
echo '     id="metadata5">' >> $inkscape_file
echo '    <rdf:RDF>' >> $inkscape_file
echo '      <cc:Work' >> $inkscape_file
echo '         rdf:about="">' >> $inkscape_file
echo '        <dc:format>image/svg+xml</dc:format>' >> $inkscape_file
echo '        <dc:type' >> $inkscape_file
echo '           rdf:resource="http://purl.org/dc/dcmitype/StillImage" />' >> $inkscape_file
echo '        <dc:title></dc:title>' >> $inkscape_file
echo '      </cc:Work>' >> $inkscape_file
echo '    </rdf:RDF>' >> $inkscape_file
echo '  </metadata>' >> $inkscape_file
echo '  <g' >> $inkscape_file
echo '     inkscape:label="Layer 1"' >> $inkscape_file
echo '     inkscape:groupmode="layer"' >> $inkscape_file
echo '     id="layer1"' >> $inkscape_file
echo '     transform="translate(0,0)">' >> $inkscape_file

sequence=1000
for filename in $directory/m*_*.map.png
do
	fileinfo=`file $filename`
	fileinfo=${fileinfo#"$filename: "}
	[ "$fileinfo" != "$common_fileinfo" ] && continue

	href=$(basename "$filename")
	absref=$(cd $(dirname "$filename"); pwd)/"$href"

	x=${filename#$directory/m}
	x=${x%_*.map.png}
	y=${filename#$directory/m*_}
	y=${y%.map.png}

	let "sequence++"
	let "x_img = ( ( x - x_min ) / map_length ) * width"
	let "y_img = total_height - ( ( ( ( y - y_min ) / map_length ) + 1 ) * height )"

	echo '    <image' >> $inkscape_file
	echo '       sodipodi:absref="'$absref'"' >> $inkscape_file
	echo '       xlink:href="'$href'"' >> $inkscape_file
	echo '       y="'$y_img'"' >> $inkscape_file
	echo '       x="'$x_img'"' >> $inkscape_file
	echo '       id="image'$sequence'"' >> $inkscape_file
	echo '       preserveAspectRatio="none"' >> $inkscape_file
	echo '       height="'$height'"' >> $inkscape_file
	echo '       width="'$width'" />' >> $inkscape_file
done

echo '  </g>' >> $inkscape_file
echo '</svg>' >> $inkscape_file

/usr/bin/inkscape $inkscape_file 2> /dev/null &


# Road Mapper Module

## Rules for groundtruth annotations on remission map images:

1. Use **InkScape**(https://inkscape.org/en/download) to draw a polyline all the way along the center of each road lane.
2. Place both the start and finish points out of the image limits, in order to provide a better fit for each polyline.
3. Place the start point of each polyline in such a way that lane orientation matches the direction from the start point to the finish point.
4. If two polylines either cross, merge or fork, then make sure to draw the *"main"* polyline before the secondary. 
5. Make each point of the polyline auto-smooth, in order to generate a cubic Bezier curve.
6. Set each polyline stroke width properly, in order to fulfill each road lane and touch the borders of the neighbor lanes.
7. Select each line stroke paint color according to the following code:
	- ![#ff0000](https://placehold.it/20x20/ff0000/?text=+) RGB(255, 000, 000) = #ff0000 = single line marking on both lane sides.
	- ![#ff007f](https://placehold.it/20x20/ff007f/?text=+) RGB(255, 000, 127) = #ff007f = single line marking on the right side and broken line marking on the left side.
	- ![#7f00ff](https://placehold.it/20x20/7f00ff/?text=+) RGB(127, 000, 255) = #7f00ff = broken line marking on the right side and single line marking on the left side.
	- ![#0000ff](https://placehold.it/20x20/0000ff/?text=+) RGB(000, 000, 255) = #0000ff = broken line marking on both lane sides.
	- ![#00ff00](https://placehold.it/20x20/00ff00/?text=+) RGB(000, 255, 000) = #00ff00 = no line marking on both lane sides
	- ![#ff7f00](https://placehold.it/20x20/ff7f00/?text=+) RGB(255, 127, 000) = #ff7f00 = single line marking on the right side and no line marking on the left side.
	- ![#7fff00](https://placehold.it/20x20/7fff00/?text=+) RGB(127, 255, 000) = #7fff00 = no line marking on the right side and single line marking on the left side.
	- ![#007fff](https://placehold.it/20x20/007fff/?text=+) RGB(000, 127, 255) = #007fff = broken line marking on the right side and no line marking on the left side.
	- ![#00ff7f](https://placehold.it/20x20/00ff7f/?text=+) RGB(000, 255, 127) = #00ff7f = no line marking on the right side and broken line marking on the left side.
8. Save the InkScape SVG file with same image name.

### Examples:

<center>

|Image Name|Remission Map | Ground Truth|Road Map |
|-----------------------|--------------|-------------|--------------|
|i7705600_-338380.png|![Remission Map](data/i7705600_-338380.png)|![Ground Truth](data/i7705600_-338380_svg.png)|![Road Map](data/r7705600_-338380_map.png)|
|i7726110_-353570.png|![Remission Map](data/i7726110_-353570.png)|![Ground Truth](data/i7726110_-353570_svg.png)|![Road Map](data/r7726110_-353570_map.png)|

</center>

 R.Carneiro:	from i7726530_-353710.png to i7758030_-363790.png
 
 R.Nascimento:	from i7705530_-338310.png to i7726460_-353990.png

**For more information look in docs/**


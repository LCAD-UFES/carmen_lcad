Rules for annotations on remission images:

1. Use InkScape to draw a polyline on the center of each road lane.
2. The lane orientation is defined by the direction from the start point to finish point of each polyline drawn.
3. The start and finish points SHOULD be placed out of the image limits, in order to provide a better fit.
4. Each point of the line should be made auto-smooth, in order to provide a cubic Bezier curve.
5. Set each line stroke width properly, in order to fulfill each road lane.
6. Select each line stroke paint color according to the following code:
	RGB(255, 0, 0)   = #ff0000 = red    = single line on both lane sides
	RGB(255, 0, 127) = #ff007f = pink   = single line on the right side and broken line on the left side
	RGB(127, 0, 255) = #7f00ff = purple = broken line on the right side and single line on the left side
	RGB(0,   0, 255) = #0000ff = blue   = broken line on both lane sides
7. Save the SVG file with same image name.

 R.Nascimento:	from i7705530_-338310.00.png to i7726460_-353990.00.png
 R.Carneiro:	from i7726530_-353710.00.png to i7758030_-363790.00.png

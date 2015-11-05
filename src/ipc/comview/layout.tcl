# -------------------------------------------------------------------- #
#
# Comview -- TCA Communication Graphic Tool
# layout.tcl
# deals with graphical layout of the comview window, creating all the
# main widgets
#
# -------------------------------------------------------------------- #

wm title . "Comview"

# wm maxsize . 1200 700
wm minsize . 1130 500

# why does the following mess w/ the geometry of Comview?
# wm resizable . f f

frame .mbar -relief raised -bd 2
frame .space -bd 2
pack .mbar -fill x
pack .space -fill x

label .space.lefttime -text ""
#pack .space.lefttime -side left -padx 120
pack .space.lefttime -side left -padx 60

# create a label above the canvas that shows the filename (the
# filename variable should be set by the C code in tcl-framework.c)
label .space.filename -relief groove -text $fname -padx 5
pack .space.filename -side left -padx 115

label .space.righttime -text ""
#pack .space.righttime -side right -padx 35
pack .space.righttime -side right -padx 15

########################################################################

frame .info
frame .info.top
frame .info.bot

frame .info.top.time
frame .info.top.tippy

button .info.top.tippy.step -text "Step" -command StepCommand
pack .info.top.tippy.step -side left
if (!$FILE_SELECTED) { .info.top.tippy.step configure -state disabled }

button .info.top.tippy.run -text "Run" -command RunCommand
pack .info.top.tippy.run -side left
if (!$FILE_SELECTED) { .info.top.tippy.run configure -state disabled }

button .info.top.tippy.unselect -text "Unselect" -command Unselect \
	-state disabled
pack .info.top.tippy.unselect -side left

button .info.top.tippy.follow -text "Scroll Lock" \
	-command FollowIncoming -relief sunken
pack .info.top.tippy.follow -side left

frame .info.zoom
pack .info.zoom -side right

frame .info.zoom.buttons
pack .info.zoom.buttons -side top

button .info.zoom.buttons.zoomin -text "Zoom In" -command zoomin
pack .info.zoom.buttons.zoomin -side left
button .info.zoom.buttons.zoomout -text "Zoom Out" -command zoomout
pack .info.zoom.buttons.zoomout -side left

scale .info.zoom.zoomscroll -from 0 -to 1 -length 4c \
	-orient horizontal -resolution 0.01 -command { zoomto }
pack .info.zoom.zoomscroll -side bottom

label .info.top.time.tlabel -text "--"
pack .info.top.time.tlabel -side right

pack .info.top.tippy -side top
pack .info.top.time -side bottom

frame .info.bot.mini
label .info.bot.mini.command
label .info.bot.mini.buf
pack .info.bot.mini -side bottom
pack .info.bot.mini.command -side left
pack .info.bot.mini.buf -side left

pack .info.bot -side bottom
pack .info.top -side top -padx 150

pack .info -side bottom -pady 20

#-----------------------------------------------------------------------
# Scrolled_Canvas
#
# procedure to create a scrolled canvas and display it.
#-----------------------------------------------------------------------

proc Scrolled_Canvas { c width height region } {
	frame $c
	canvas $c.canvas -width $width -height $height \
		-scrollregion $region \
		-xscrollcommand [list $c.xscroll set] \
		-yscrollcommand [list $c.yscroll set] \
		-background grey \
		-relief sunken
	scrollbar $c.xscroll -orient horizontal \
		-command CanvasScrollX
	scrollbar $c.yscroll -orient vertical \
		-command CanvasScrollY
	pack $c.xscroll -side bottom -fill x
	pack $c.yscroll -side right -fill y
	pack $c.canvas -side left -fill both -expand true
	pack $c -side right -fill both -expand true
	return $c.canvas
}

#########################################################################
########################## CANVAS LAYOUT ################################

frame .a

# Create a  canvas to be used for the module labels:
frame .a.mod
canvas .a.mod.names -width $MODULE_CANVAS_WIDTH -height $CANVAS_SHOW_HEIGHT
pack .a.mod.names -padx 4 -side left -fill y

set foo [list 0 0 $CANVAS_WIDTH $CANVAS_HEIGHT]

Scrolled_Canvas .a.c $CANVAS_SHOW_WIDTH $CANVAS_SHOW_HEIGHT $foo

pack .a
pack .a.mod
pack .a.mod.names

.a.c.canvas configure -rel sunken

set place .a.c.canvas

# this line is meant to make visible the location of the left edge of
# the canvas when the user has zoommed out to a level where x=0 on the
# canvas is shown at a position greater than x=0.
$place create line -1 0 -1 1500 -fill white


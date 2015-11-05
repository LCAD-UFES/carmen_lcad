# The vistool is not currently being used

#########################################################################
###################### Visual Alignment Tool ############################

# The visual alignment tool is designed for manipulation of the
# graphical canvas display. Primary features are: capability to zoom
# in or out from a given point, a single-message step in either
# direction, a tool for assisting in visually comparing the timing
# alignment of messages.

if (0) {

set top_alignment [$place create oval 10 10 16 16 -fill white \
	-outline blue -tags "time_alignment ta_move"]
set bottom_alignment [$place create oval 10 370 16 376 -fill white \
	-outline blue -tags "time_alignment ta_move"]
$place create line 12 16 12 370 -fill blue -tags "time_alignment ta_move"

# Create the rectangle with "+" for Zoomming In
$place create rectangle 17 369 26 377 -tags time_alignment
$place create text 22 373 -text "+" -tags "time_alignment ta_zin"
$place bind ta_zin <Button-1> {
    zoomin
}

# Create the rectangle with "-" for Zoomming Out
$place create rectangle 0 369 8 377 -tags time_alignment
$place create text 5 373 -text "-" -tags "time_alignment ta_zout"
$place bind ta_zout <Button-1> {
    zoomout
}

}

if (0) {

# Create the arrow for stepping forward
$place create line 20 13 26 13 -arrow last -tags "time_alignment ta_fwd"
$place bind ta_fwd <Button-1> {
    $place move time_alignment [expr $Time_Scale_Factor*$Scale_Factor] 0
}

# Create the arrow for stepping back
$place create line 5 13 1 13 -arrow last -tags "time_alignment ta_bck"
$place bind ta_bck <Button-1> {
    $place move time_alignment [expr -$Time_Scale_Factor*$Scale_Factor] 0
}

}

# The next four bindings are for basic control of the visual alignment
# tool: creating an outline on the circles, and moving the line.

$place bind ta_move <Any-Enter> {
    $place itemconfigure $top_alignment -outline white
    $place itemconfigure $bottom_alignment -outline white
}

$place bind ta_move <Any-Leave> {
    .a.c.canvas itemconfigure $top_alignment -outline blue
    .a.c.canvas itemconfigure $bottom_alignment -outline blue
}

$place bind ta_move <Button-1> {
    set cur_alignmentX %x
    $place raise time_alignment
}

$place bind ta_move <B1-Motion> {
    $place move time_alignment [expr %x-$cur_alignmentX] 0
    set cur_alignmentX %x
    set time_position %x
}

# This binding is for allowing the second mouse button to result in
# centering the canvas around the current position of the visual
# alignment tool:

$place bind ta_move <Button-2> {
    set cur_alignmentX %x
    $place raise time_alignment
    debugp "time_position=$time_position"
    Canvas_Moveto [expr $time_position/$CANVAS_WIDTH]
    set time_position \
	    [expr ($time_position-$CANVAS_SHOW_WIDTH/2)/$CANVAS_WIDTH]

}


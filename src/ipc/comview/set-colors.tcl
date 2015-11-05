#-----------------------------------------------------------------------
# set-colors.tcl
# Code to change the colors of the different message types
#
# When a communication graphic is created on the canvas one of the
# tags that it is given is "srcX" or "rcvX", depending on whether the
# graphic identifies the source of a message or the destination of a 
# message. The number X is the message type (as defined in 
# tca-header.tcl).
#
# This code will pop-up a window which allows you to modify the color
# settings for any message pairs (ie. the source and destination
# colors for a Query message).
#
# When the name of a color is known the RGB values are given in the
# file rgb.txt, which can usually be found in /usr/lib/X11 on suns.
# 
#-----------------------------------------------------------------------

proc Change_Color { msg_type name } {
    global type task_color rcv_color orig_src orig_dst

    set type $msg_type
    set window .colobar$msg_type
    
    toplevel $window
    wm title $window "Change $name Colors"

    frame $window.src
    frame $window.dst
    frame $window.controls

    pack $window.src 
    pack $window.dst
    pack $window.controls

    # ----- Buttons -----

    set set_color_command \
	    "Set_Colors $msg_type $window $name"
    button $window.controls.set -text "Set" \
	    -command $set_color_command

    pack $window.controls.set -side left
    button $window.controls.save -text "Save" \
	    -command { Save_Colors "colors.tcl" }
    pack $window.controls.save -side left 

    set cancel_command "Cancel $window src$msg_type $task_color($msg_type) rcv$msg_type $rcv_color($msg_type)"
    button $window.controls.cancel -text "Cancel" \
	    -command $cancel_command
    pack $window.controls.cancel -side left

    set reset_command "Reset $window.src $window.dst src$msg_type $task_color($msg_type) rcv$msg_type $rcv_color($msg_type)"
    button $window.controls.reset -text "Reset" -command $reset_command
    pack $window.controls.reset -side left

    # ----- End Buttons -----

    set col $task_color($msg_type)
    Make_Color_Control "task_color($msg_type)" \
	    $window.src "src$msg_type" "SOURCE" $col

    set col $rcv_color($msg_type)
    Make_Color_Control "rcv_color($msg_type)" \
	    $window.dst "rcv$msg_type" "DESTINATION" $col

    set orig_src($msg_type) $task_color($msg_type)
    set orig_dst($msg_type) $rcv_color($msg_type)
}

proc Cancel { window src src_orig rcv rcv_orig } {
    destroy $window
    Reset_Colors $src $src_orig $rcv $rcv_orig
}

proc Reset_Colors { src src_orig rcv rcv_orig } {

    .a.c.canvas itemconfigure $src -fill $src_orig
    .a.c.canvas itemconfigure $src -outline $src_orig

    .a.c.canvas itemconfigure $rcv -fill $rcv_orig
    .a.c.canvas itemconfigure $rcv -outline $rcv_orig
}

proc Reset { src_window rcv_window src src_orig rcv rcv_orig } {

    Reset_Colors $src $src_orig $rcv $rcv_orig
    
    $src_window.red set [expr 0x[string range $src_orig 1 2]]
    $src_window.green set [expr 0x[string range $src_orig 3 4]]
    $src_window.blue set [expr 0x[string range $src_orig 5 6]]

    $rcv_window.red set [expr 0x[string range $rcv_orig 1 2]]
    $rcv_window.green set [expr 0x[string range $rcv_orig 3 4]]
    $rcv_window.blue set [expr 0x[string range $rcv_orig 5 6]]
}

# Make_Color_Control will create three sliders and a canvas which
# initially displays the color specified as the fourth argument
# (col). This color can be modified by moving the slider bars which
# correspond to the Red, Green, and Blue component of the color.

proc Make_Color_Control { var location item words col } {

    set r [expr 0x[string range $col 1 2]]
    set g [expr 0x[string range $col 3 4]]
    set b [expr 0x[string range $col 5 6]]

    set c "set_red $var $location $item"
    scale $location.red -from 0 -to 255 -length 3c \
	    -orient vertical -resolution 1 -command $c
    $location.red set $r

    set c "set_green $var $location $item"
    scale $location.green -from 0 -to 255 -length 3c \
	    -orient vertical -resolution 1 -command $c
    $location.green set $g

    set c "set_blue $var $location $item"
    scale $location.blue -from 0 -to 255 -length 3c \
	    -orient vertical -resolution 1 -command $c
    $location.blue set $b

    canvas $location.example -width 4c -height 3c
    
    $location.example create text 2c 1.5c -text $words

    pack $location.red -side left 
    pack $location.green -side left
    pack $location.blue -side left
    pack $location.example
}

proc change_rgb { var name item r g b } {
    global task_color rcv_color

    set r [hex $r]
    set g [hex $g]
    set b [hex $b]

    set gcolor #$r$g$b

    $name.example configure -bg $gcolor
    .a.c.canvas itemconfigure $item -fill $gcolor
    .a.c.canvas itemconfigure $item -outline $gcolor
    set $var $gcolor
}

proc set_red { var name item amount } {
    change_rgb $var $name $item \
	    $amount [$name.green get] [$name.blue get]
}

proc set_green { var name item amount } {
    change_rgb $var $name $item \
	    [$name.red get] $amount [$name.blue get]
}

proc set_blue { var name item amount } {
    change_rgb $var $name $item \
	    [$name.red get] [$name.green get] $amount
}

# Set_Colors

proc Set_Colors { msg_type window name } {
    global task_color rcv_color

    # Change the colors in the menu options:

    set source_index [.mbar.colors.menu index "$name sent"]
    set dest_index [.mbar.colors.menu index "$name receive"]

    .mbar.colors.menu entryconfigure $source_index\
	    -background $task_color($msg_type)
    .mbar.colors.menu entryconfigure $dest_index \
	    -background $rcv_color($msg_type)

    # Get rid of the colors window:
    destroy $window
}

# Save the color settings to a file

proc Save_Colors { filename } {
    global task_color rcv_color
    global REPLY_MESSAGE QUERY_MESSAGE INFORM_MESSAGE COMMAND_MESSAGE
    global BROADCAST_MESSAGE GOAL_MESSAGE
    
    set fd [open $filename "w"]

    puts $fd "# -------------------------------------------------------------------
# The colors in this file can be modified via the menus in Comview.
#
# -------------------------------------------------------------------

set task_color(\$REPLY_MESSAGE) $task_color($REPLY_MESSAGE)
set task_color(\$QUERY_MESSAGE) $task_color($QUERY_MESSAGE)
set task_color(\$INFORM_MESSAGE) $task_color($INFORM_MESSAGE)
set task_color(\$COMMAND_MESSAGE) $task_color($COMMAND_MESSAGE)
set task_color(\$BROADCAST_MESSAGE) $task_color($BROADCAST_MESSAGE)
set task_color(\$GOAL_MESSAGE) $task_color($GOAL_MESSAGE)
    
set rcv_color(\$QUERY_MESSAGE) $rcv_color($QUERY_MESSAGE)
set rcv_color(\$INFORM_MESSAGE) $rcv_color($INFORM_MESSAGE)
set rcv_color(\$COMMAND_MESSAGE) $rcv_color($COMMAND_MESSAGE)
set rcv_color(\$BROADCAST_MESSAGE) $rcv_color($BROADCAST_MESSAGE)
set rcv_color(\$GOAL_MESSAGE) $rcv_color($GOAL_MESSAGE)
   
set PEND_BAR_COLOR #ffa500"

    close $fd
}

# ----------------------------------------------------------------------
# Some utility hex routines

proc hex_digit { number } {
    if ($number<10) { return $number }
    if ($number==10) { return a }
    if ($number==11) { return b }
    if ($number==12) { return c }
    if ($number==13) { return d }
    if ($number==14) { return e }
    if ($number==15) { return f }
}

# hex converts an integer (0..255) to a hex number (#00..#ff)

proc hex { number } {
    set a [hex_digit [expr $number % 16]]
    set b [hex_digit [expr $number / 16]]
    return $b$a
}

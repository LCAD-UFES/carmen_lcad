# -------------------------------------------------------------------- #
#
# Comview -- TCA Communication Graphic Tool
# Greg Whelan, whelan@cs.cmu.edu
# Carnegie Mellon University
#
# comview.tcl Main source
#
# -------------------------------------------------------------------- #

########################################################################
######################## CANVAS MANIPULATION ###########################

set scroll_offset 0

#-----------------------------------------------------------------------
# Canvas_Change_Time
# If the canvas is scaled or shifted then the time corresponding to
# the leftmost visible side of the canvas needs to be determined.  
#-----------------------------------------------------------------------
proc Canvas_Change_Time {} {
    global scroll_offset CANVAS_WIDTH DISPLAY_TIME Scale_Factor
    global CANVAS_SHOW_WIDTH

    set b [lindex [.a.c.canvas xview] 0]
    set scroll_offset [expr $CANVAS_WIDTH*$b]
    if ($DISPLAY_TIME) {
	.space.lefttime configure \
		-text [Time_String [expr $scroll_offset/$Scale_Factor]]
	.space.righttime configure -text [Time_String \
		[expr ($scroll_offset+$CANVAS_SHOW_WIDTH)/$Scale_Factor]]
    }
}


#-----------------------------------------------------------------------
# Whenever the canvas is scrolled the scroll_offset variable needs to
# be updated.
#-----------------------------------------------------------------------
proc CanvasScrollX { a b { c d } } {

    if ([string compare $a moveto]==0) {
	.a.c.canvas xview $a $b
    } else {
	.a.c.canvas xview $a $b $c
    }
    
    Canvas_Change_Time
}


#-----------------------------------------------------------------------
# Whenever the canvas is scrolled the scroll_offset variable needs to
# be updated.
#-----------------------------------------------------------------------
proc CanvasScrollY { a b { c d } } {

    if ([string compare $a moveto]==0) {
	.a.c.canvas yview $a $b
	.a.mod.names yview $a $b
    } else {
	.a.c.canvas yview $a $b $c
	.a.mod.names yview $a $b $c
    }
}


#-----------------------------------------------------------------------
# Canvas_Left_Edge returns the position within the canvas that is
# currently at the leftmost visible side of the display.
#-----------------------------------------------------------------------

proc Canvas_Left_Edge {} {
    global CANVAS_WIDTH Scale_Factor

    set current_view [.a.c.canvas xview]
    return [expr [lindex $current_view 0] * $CANVAS_WIDTH / $Scale_Factor]
}


#-----------------------------------------------------------------------
# Canvas_Right_Edge returns the position within the canvas that is
# currently at the rightmost visible side of the display.
#-----------------------------------------------------------------------

proc Canvas_Right_Edge {} {
    global CANVAS_WIDTH Scale_Factor

    set current_view [.a.c.canvas xview]
    return [expr [lindex $current_view 1] * $CANVAS_WIDTH / $Scale_Factor]
}


#-----------------------------------------------------------------------
# Canvas_Center_Around
#-----------------------------------------------------------------------
proc Canvas_Center_Around { position } {
    global CANVAS_WIDTH Scale_Factor CANVAS_SHOW_WIDTH

    set x [expr ($position-$CANVAS_SHOW_WIDTH/2)*$Scale_Factor/$CANVAS_WIDTH]
    
    if ($x<0) { set x 0 }
    if ($x>1) { set x 1 }

    Canvas_Moveto $x
}


#-----------------------------------------------------------------------
# Canvas_Moveto
#-----------------------------------------------------------------------
proc Canvas_Moveto { position } {
    .a.c.canvas xview moveto $position
    Canvas_Change_Time
}


#-----------------------------------------------------------------------
# Canvas_Scroll
#-----------------------------------------------------------------------
proc Canvas_Scroll { amt unit } {
    .a.c.canvas xview scroll $amt $unit
    Canvas_Change_Time
}

#-----------------------------------------------------------------------
# Unselect_Current
#-----------------------------------------------------------------------
proc Unselect_Current {} {
    global CurrentShowAll current_highlight cur

    set current_highlight 0
    set CurrentShowAll "0"

    .a.c.canvas delete module_call_arrows
}

#-----------------------------------------------------------------------
# Unselect
#-----------------------------------------------------------------------
proc Unselect {} {
    global current_highlight

    Unselect_Current
    set current_highlight 0
    .info.top.tippy.unselect configure -state disabled
}


#-----------------------------------------------------------------------
# Update_Visible_Time
#-----------------------------------------------------------------------
proc Update_Visible_Time {} {
    global DISPLAY_TIME scroll_offset Scale_Factor CANVAS_SHOW_WIDTH
    
    if ($DISPLAY_TIME==1) {
	.space.lefttime configure \
		-text [Time_String [expr $scroll_offset/$Scale_Factor]]
	.space.righttime configure -text [Time_String \
		[expr ($scroll_offset+$CANVAS_SHOW_WIDTH)/$Scale_Factor]]
    }
}

#-----------------------------------------------------------------------
# Toggle_Visible_Time
#-----------------------------------------------------------------------
proc Toggle_Visible_Time {} {
    global DISPLAY_TIME

    set DISPLAY_TIME [expr 1-$DISPLAY_TIME]
    if ($DISPLAY_TIME==0) {
	.space.lefttime configure -text ""
	.space.righttime configure -text ""
    } else {
	Update_Visible_Time
    }
}

########################################################################
##################### ZOOM AND RESIZING CANVAS #########################


#-----------------------------------------------------------------------
#
# grow_canvas will expand the size of the canvas and add vertical
# lines for the time scale.
#
#-----------------------------------------------------------------------

proc grow_canvas {amt} {
    global Scale_Factor Time_Scale_Factor place NUMBER_OF_LINES
    global CANVAS_WIDTH CANVAS_HEIGHT
    
    debugp "about to grow_canvas $amt"

    set CANVAS_WIDTH [expr $CANVAS_WIDTH+$amt]
    set foo [list 0 0 $CANVAS_WIDTH $CANVAS_HEIGHT]
    .a.c.canvas configure -scrollregion $foo

    if (0) {
    for {set loop $NUMBER_OF_LINES } {$loop < [expr $NUMBER_OF_LINES+($amt/$Time_Scale_Factor)] } { incr loop } { 

	$place create line [expr $loop*$Scale_Factor*$Time_Scale_Factor] \
		0 \
		[expr $loop*$Scale_Factor*$Time_Scale_Factor] \
		550 -fill lightgrey -tags "time_scale"
	$place lower time_scale
    }
}
    debugp "done with grow_canvas"
}


#-----------------------------------------------------------------------
# 
# scale_canvas will expand all the items on the canvas by using the
# canvas command "scale"
#
#-----------------------------------------------------------------------

proc scale_canvas { amt } {
    global Scale_Factor time_position place Time_Scale_Factor
    global current_highlight CANVAS_WIDTH highlight_multiple

    set save_ch $current_highlight
    # we don't scale the call arrows so delete them -- should redraw
    Unselect_Current

    # resize all the items on the canvas about the line x=0
    $place scale single_node 0 0 $amt 1
    $place scale canvas_item 0 0 $amt 1
    $place scale time_scale 0 0 $amt 1
    $place scale event_line 0 0 $amt 1
    $place scale event_line 0 0 $amt 1
    $place scale pending_node 0 0 $amt 1

    set Scale_Factor [expr $Scale_Factor*$amt]

    # Depending on the value of the Scale_Factor we change the
    # interval between the time scale lines.

    set Time_Scale_Factor [expr $Time_Scale_Factor*$amt]
    if ($Time_Scale_Factor<5) {
	set Time_Scale_Factor [expr $Time_Scale_Factor*10]
	$place scale time_scale 0 0 10 1
	$place lower time_scale
    } elseif ($Time_Scale_Factor>50) {
	set Time_Scale_Factor [expr $Time_Scale_Factor/10]
	$place scale time_scale 0 0 0.1 1
	$place lower time_scale
    }

    # when we scale the canvas the width must be increased (for zoom
    # in) or decreased (for zoom out) to ensure proper coverage of the
    # region by the scrollbar.
    grow_canvas [expr ($CANVAS_WIDTH*$amt)-$CANVAS_WIDTH]

    Canvas_Change_Time

    set current_highlight $save_ch
    # re-highlight any activity which was highlighted before the zoom
    if ($current_highlight>0) { Basic_Highlight $current_highlight }

    if ($highlight_multiple>0) {
	# need to fill this in
    }
}

#-----------------------------------------------------------------------
# zoom
#
#-----------------------------------------------------------------------

proc zoom { zamt } {
    global place CANVAS_WIDTH Scale_Factor CANVAS_SHOW_WIDTH
    global current_highlight waiting_history waiting_call
    global CurrentShowAll

    if ([string compare $CurrentShowAll "0"]==0) { 
	set restore_all 0
    } else {
	set restore_all 1
	set restore_name $CurrentShowAll
    }

    # if there's an item currently select then we zoom in around that object
    if ($current_highlight>0) {
	# determine the center of the currently selected object
	set position [expr ([lindex [$place bbox $current_highlight] 0]+\
		[lindex [$place bbox $current_highlight] 2])]
	set position [expr $position/(2*$Scale_Factor)]
	
	set canvas_pos [expr ($position*$Scale_Factor-(350/$zamt))]
	set canvas_pos [expr $canvas_pos/$CANVAS_WIDTH]

    } else {
	# this is for the case where there is no object currently selected. 
	# we zoom in about the center of the current canvas view.

	# determine the position at which the user is currently focussed
	set position [expr ($CANVAS_WIDTH*[lindex [$place xview] 0])]
	# and what that position will be after the scale
	# (the canvas xview is specified in terms of the position of
	# the left side of the canvas).
	set position [expr $position+350-(350/$zamt)]
	
	set canvas_pos [expr $position/$CANVAS_WIDTH]
    }

    scale_canvas $zamt
    
    Canvas_Moveto $canvas_pos

    if ($restore_all) { Show_All_Message $restore_name }
}

proc zoomin {} {
    global Zoom_Amount

    zoom $Zoom_Amount
}

proc zoomout {} {
    global Zoom_Amount
    zoom [expr 1/$Zoom_Amount]
}

# zoomto 0=zoom all the way out. 1=fixed zoom in level

set started_hack 0

proc zoomto { x } {
    global Scale_Factor CANVAS_WIDTH CANVAS_SHOW_WIDTH started_hack

    if ($started_hack>0) {
	set out [expr $CANVAS_SHOW_WIDTH/$CANVAS_WIDTH]
	set in [expr 4.0/$Scale_Factor]

	set scale_amount [expr $x*$out+(1.0-$x)*$in/200.0]
	zoom $scale_amount
    } else {
	# this is neccessary because the scale widget will execute it's
	# associated command as soon as the widget appears on the
	# screen. Since the initial value may not be correct we
	# prevent this from happenning.
	incr started_hack
    }
}

##########################################################################
######################## Canvas Time Display  ############################

# The hours, mins, secs and hsecs global variables are only set from
# within C (we only create a binding that is globally visible so that
# the C code can modify them). They hold the time at which the first
# message in the log file occurred.

set hours 0
set mins 0
set secs 0
set hsecs 0

# these procedures are called from C when the first message with a
# time-stamp is parsed.

proc set_hours { x } {
    global hours
    set hours $x
}

proc set_mins { x } {
    global mins
    set mins $x
}

proc set_secs { x } {
    global secs
    set secs $x
}

proc set_hsecs { x } {
    global hsecs
    set hsecs $x
}

proc Time_List { offset } {
    global hours mins secs hsecs absolute Scale_Factor

    if ($absolute) {
	# Time should be absolute
	set mot_hsecs $hsecs
	set mot_secs $secs
	set mot_mins $mins
	set mot_hours $hours
    } else {
	# Time should be relative to first message
	set mot_hsecs 0
	set mot_secs 0
	set mot_mins 0
	set mot_hours 0
    }

    # Use that value to determine what the absolute time is
    set hs [expr $mot_hsecs+int($offset/10)]
    set s [expr $mot_secs+($hs/100)]
    set m [expr $mot_mins+($s/60)]
    set h [expr $mot_hours+($m/60)]
    set hs [expr round($hs) % 100]
    set s [expr round($s) % 60]
    set m [expr round($m) % 60]
    if ($hs<10) { set hs "0$hs" }
    if ($s<10) { set s "0$s" }
    if ($m<10) { set m "0$m" }
    
    return [list $h $m $s $hs]
}

proc Time_String { offset } {
    set tl [Time_List $offset]
    set h [lindex $tl 0]
    set m [lindex $tl 1]
    set s [lindex $tl 2]
    set hs [lindex $tl 3]

    return "$h:$m:$s.$hs"
}

# changing this to 1 will enable the display of the current
# time-coordinate below the mouse cursor.
set mouse_time 0

########################################################################
######################### DEBUGGING UTILS ##############################

set pause 0

proc debugp { output } {
    global pause
#    puts $output
#    update
#    if ($pause==1) { gets stdin }
}

########################################################################


proc load_file {} {
    global filename fname

    set filename [FSBox]

    set fname [string range $filename \
	    [expr [string last "/" $filename]+1] \
	    [string length $filename]]

    .info.top.tippy.step configure -state normal
    .info.top.tippy.run configure -state normal

    .space.filename configure -relief groove -text $fname -padx 5
    
    C_Load_File $filename
}


#-----------------------------------------------------------------------
# 
# FollowIncoming will toggle the option of having the graphics display
# continually scroll when new data is displayed. It is toggled with the
# "Scroll Lock" button.
#
#-----------------------------------------------------------------------

proc FollowIncoming {} {
    global follow_production
    set follow_production [expr 1-$follow_production]
    if ($follow_production) {
	.info.top.tippy.follow configure -relief sunken
    } else {
	.info.top.tippy.follow configure -relief raised
    }
}

########################################################################
########################### TIME SCALE #################################

set NUMBER_OF_LINES [expr $CANVAS_WIDTH / $Time_Scale_Factor]
set NUMBER_OF_LINES 1000

if (0) {

for {set loop 1 } {$loop < $NUMBER_OF_LINES} { incr loop } { 
    
    set unit [expr $loop % 10]
    if ($unit==0) {
	    set unit [expr $loop % 100]
	if ($unit==0) {
	    set unit 100
	} else {
	    set unit 10
	}
    } else {
	set unit 1
    }

    $place create line [expr $loop*$Scale_Factor*$Time_Scale_Factor] \
	    0 \
	    [expr $loop*$Scale_Factor*$Time_Scale_Factor] \
	    550 -fill lightgrey -tags "time_scale ts$unit"
#	    550 -fill black -tags "time_scale ts$unit"

    set time_val [expr $loop % 100]
    if ($time_val<10) {
	set time_text "0$time_val"
    } else {
	set time_text "$time_val"
    }

# Labels for the time scale (not very usable)
#    $place create text \
#	    [expr $loop*$Scale_Factor*$Time_Scale_Factor] 16 \
#	    -text $time_text \
#	    -tags time_scale
}

}

#-----------------------------------------------------------------------
# draw_time_grid
#
#-----------------------------------------------------------------------

proc draw_time_grid {} {
    global Scale_Factor Time_Scale_Factor scroll_offset CANVAS_WIDTH
    global place

    # determine the start and end X coordinates currently visible
    # within the canvas.
    set start [expr round($scroll_offset)]
    set stop [expr round($scroll_offset+$CANVAS_WIDTH)]

    for {set loop $start } {$loop<$stop} { incr loop 10} {
        $place create line $loop \
		0 \
		$loop \
		550 -fill lightgrey -tags "time_grid"
    }
}

##########################################################################
################### SAVE TRANSCRIPT TO POSTSCRIPT ########################

proc savepost { filename } {
    global place OUTLINE COLOR_LIST

    if ($OUTLINE) {
	# if the OUTLINE option is set then we configure all the Tk
	# objects on the canvas to have a black outline (for visual
	# purposes). the outlines will be removed after saving the
	# canvas to postscript, and restored to their original color.
	$place itemconfigure single_node -outline black
    }

    $place postscript -file $filename -rotate true

    if ($OUTLINE) {
	# remove outlines
	for {set loop 0} {($loop < [llength $COLOR_LIST])} {incr loop} {
	    set f [lindex $COLOR_LIST $loop]
	    $place itemconfigure node$f -outline $f
	}
    }
}

#-----------------------------------------------------------------------
# save_to_postscript
# this function creates a menu option for entering a filename to save
# the canvas to postscript. the actual saving to postscript is done by
# the function savepost.
#-----------------------------------------------------------------------

proc save_to_postscript {} {
    global raw_filename

    toplevel .save
    wm title .save "Save to Postscript"

    set fn [pwd]
    append fn "/"
    append fn "$raw_filename"
    append fn ".ps"
    
    text .save.filename -width 50 -height 1
    .save.filename insert 1.0 $fn
    pack .save.filename

    button .save.ok -text "Ok" \
	    -command { save_command }
    button .save.cancel -text "Cancel" -command { destroy .save }

    pack .save.ok -side left
    pack .save.cancel -side right
}

#-----------------------------------------------------------------------
# save_command is called when the "Ok" button is hit from the "Save to
# Postscript" menu.
#-----------------------------------------------------------------------

proc save_command {} {
    savepost [.save.filename get 1.0 1.end]
    destroy .save
}

##########################################################################
############################# Goto Fns ###################################

# The following functions are provided to jump to a specific time
# (absolute or relative), or a specific message.

#-----------------------------------------------------------------------
# Goto_Time
#-----------------------------------------------------------------------

proc Goto_Time { h m s hs } {
    global hours mins secs hsecs Scale_Factor CANVAS_WIDTH
    
    # calculate the difference in hundreths of a seconds between the
    # time to goto and the time stamp of the first message in the 
    # log file.
    set goto_t [expr ($hs+(((($h*60)+$m)*60)+$s)*100)]
    set first_time [expr ($hsecs+(((($hours*60)+$mins)*60)+$secs)*100)]
    set time_difference [expr $goto_t - $first_time]

    # scroll the canvas so the given time is now at the leftmost part 
    # of the canvas.
    set scroll_amt \
	    [expr ($time_difference*10.0*$Scale_Factor) / $CANVAS_WIDTH]
    
    CanvasScrollX moveto $scroll_amt
}

proc Goto_Time_Menu {} {
    global absolute time_count Goto_Time_Menu_Exists

    if ($Goto_Time_Menu_Exists==0) {

	toplevel .gt
	wm title .gt "Go To Time"
	
	frame .gt.top

	set st [Time_List 0]
	set h [lindex $st 0]
	set m [lindex $st 1]
	set s [lindex $st 2]
	set hs [lindex $st 3]

	text .gt.foo

	set text_font [lindex [.gt.foo configure -font] 4]

	frame .gt.top.start

	label .gt.top.start.hours -height 1 -width 2 -text $h \
		-relief groove -font $text_font
	label .gt.top.start.a -text ":"
	label .gt.top.start.mins -height 1 -width 2 -text $m \
		-relief groove -font $text_font
	label .gt.top.start.b -text ":"
	label .gt.top.start.secs -height 1 -width 2 -text $s \
		-relief groove -font $text_font
	label .gt.top.start.c -text "."
	label .gt.top.start.hsecs -height 1 -width 2 -text $hs \
		-relief groove -font $text_font
	
	pack .gt.top.start.hours -side left
	pack .gt.top.start.a -side left
	pack .gt.top.start.mins -side left
	pack .gt.top.start.b -side left
	pack .gt.top.start.secs -side left
	pack .gt.top.start.c -side left
	pack .gt.top.start.hsecs -side left

	pack .gt.top -side top
	pack .gt.top.start -side top

	frame .gt.t

	text .gt.t.hours -height 1 -width 2 -wrap none
	.gt.t.hours insert 1.0 $h
	bind .gt.t.hours <KeyPress> { .gt.t.hours delete 1.1 end }
	bind .gt.t.hours <BackSpace> {}
	bind .gt.t.hours <Tab> {}

	label .gt.t.a -text ":"
	text .gt.t.mins -height 1 -width 2 -wrap word
	.gt.t.mins insert 1.0 $m
	bind .gt.t.mins <KeyPress> { .gt.t.mins delete 1.1 end }
	bind .gt.t.mins <BackSpace> {}
	bind .gt.t.mins <Tab> {}

	label .gt.t.b -text ":"
	text .gt.t.secs -height 1 -width 2 -wrap char
	.gt.t.secs insert 1.0 $s
	bind .gt.t.secs <KeyPress> { .gt.t.secs delete 1.1 end }
	bind .gt.t.secs <BackSpace> {}
	bind .gt.t.secs <Tab> {}

	label .gt.t.c -text "."
	text .gt.t.hsecs -height 1 -width 2
	.gt.t.hsecs insert 1.0 $hs
	bind .gt.t.hsecs <KeyPress> { .gt.t.hsecs delete 1.1 end }
	bind .gt.t.hsecs <BackSpace> {}
	bind .gt.t.hsecs <Tab> {}

	pack .gt.t -side top

	pack .gt.t.hours -side left
	pack .gt.t.a -side left
	pack .gt.t.mins -side left
	pack .gt.t.b -side left
	pack .gt.t.secs -side left
	pack .gt.t.c -side left
	pack .gt.t.hsecs -side left

	set st [Time_List $time_count]
	set h [lindex $st 0]
	set m [lindex $st 1]
	set s [lindex $st 2]
	set hs [lindex $st 3]

	frame .gt.bot
	frame .gt.bot.end

	label .gt.bot.end.hours -height 1 -width 2 -text $h \
		-relief groove -font $text_font
	label .gt.bot.end.a -text ":"
	label .gt.bot.end.mins -height 1 -width 2 -text $m \
		-relief groove -font $text_font
	label .gt.bot.end.b -text ":"
	label .gt.bot.end.secs -height 1 -width 2 -text $s \
		-relief groove -font $text_font
	label .gt.bot.end.c -text "."
	label .gt.bot.end.hsecs -height 1 -width 2 -text $hs \
		-relief groove -font $text_font
	
	pack .gt.bot.end.hours -side left
	pack .gt.bot.end.a -side left
	pack .gt.bot.end.mins -side left
	pack .gt.bot.end.b -side left
	pack .gt.bot.end.secs -side left
	pack .gt.bot.end.c -side left
	pack .gt.bot.end.hsecs -side left

	pack .gt.bot -side top
	pack .gt.bot.end -side top

	frame .gt.bot.buttons
	button .gt.bot.buttons.go -text "Go" \
		-command { Goto_Time_Stub }

	button .gt.bot.buttons.cancel -text "Cancel/Done" \
		-command { Destroy_Goto_Time_Menu }

	pack .gt.bot.buttons -side bottom
	pack .gt.bot.buttons.go -side left
	pack .gt.bot.buttons.cancel -side right
    }
}

proc Goto_Time_Stub {} {
    catch { Goto_Time \
	    [expr [.gt.t.hours get 1.0 1.2]] \
	    [expr [.gt.t.mins get 1.0 1.2]] \
	    [expr [.gt.t.secs get 1.0 1.2]] \
	    [expr [.gt.t.hsecs get 1.0 1.2]] }
}

proc Destroy_Goto_Time_Menu {} {
    global Goto_Time_Menu_Exists
    destroy .gt
    set Goto_Time_Menu_Exists 0
}


# rlz = remove leading zeroes
# Given a number (Tcl represents as a string) remove the leading
# zero's (ie. "000123" ==> "123")

proc rlz { number } {
    if (([string compare [string index $number 0] "0"]==0)&&([string length $number]>1)) {
	return [string range $number 1 end]
    }
    return $number
}

proc Goto_Time_Unparsed { time } {
    # need to parse the time string into the 4 components
    set result 0
    set loc1 [string first ":" $time]
    incr result [catch {set h [rlz [string range $time 0 [expr $loc1-1]]]}]
    incr result [catch {set m [rlz [string range $time \
	    [expr $loc1+1] [expr $loc1+2]]]}]
    incr result [catch {set s [rlz [string range $time \
	    [expr $loc1+4] [expr $loc1+5]]]}]
    incr result [catch {set u [rlz [string range $time \
	    [expr $loc1+7] [expr $loc1+8]]]}]

    if ($result==0) { Goto_Time $h $m $s $u }
}

#-----------------------------------------------------------------------
# 
# Goto_Msg takes a message number and will highlight the activity in
# the canvas window and highlight the appropriate line in the listbox
# (centering the current view of both widgets around the newly 
# selected item).
#
#-----------------------------------------------------------------------

proc Goto_Msg { msg_number } {
    global message_graphic msg_count current_highlight

    if (($msg_number>0)&&($msg_number<$msg_count)) {
	Unselect_Current
	set item $message_graphic($msg_number)
	if ($item>=0) {
	    set current_highlight $item
	    Basic_Highlight $current_highlight
	    Center_In_Canvas $current_highlight
	}
	Listbox_Select $msg_number
    }
}

proc Goto_Msg_Action {} {
    catch { Goto_Msg [expr [.gm.top.msg get 1.0 1.5]] }
    .gm.top.msg delete 1.0 end
}

proc Goto_Msg_Menu {} {
    global msg_count Goto_Msg_Menu_Exists

    if ($Goto_Msg_Menu_Exists==0) {

	set Goto_Msg_Menu_Exists 1

	toplevel .gm
	wm title .gm "Go To Message"
	
	frame .gm.top
	frame .gm.bottom
	label .gm.top.label -text "Message Number:"
	text .gm.top.msg -height 1 -width 4 -wrap none
	bind .gm.top.msg <KeyPress> { .gm.top.msg delete 1.3 end }
	bind .gm.top.msg <Tab> {}
	bind .gm.top.msg <BackSpace> {}
	bind .gm.top.msg <Return> { Goto_Msg_Action }

	set max [expr $msg_count-1]
	label .gm.top.max -text "(max $max)"
	button .gm.bottom.go -text "Go" -command { Goto_Msg_Action }
	button .gm.bottom.cancel -text "Cancel/Done" \
		-command { Destroy_Goto_Msg_Menu }
	
	pack .gm.top -side top
	pack .gm.bottom -side bottom
	pack .gm.top.label -side left
	pack .gm.top.msg -side left
	pack .gm.top.max -side left
	pack .gm.bottom.go -side left
	pack .gm.bottom.cancel -side left
    }
}

proc Destroy_Goto_Msg_Menu {} {
    global Goto_Msg_Menu_Exists
    destroy .gm
    set Goto_Msg_Menu_Exists 0
}

#-----------------------------------------------------------------------
# Goto_Msg_No_Del
#-----------------------------------------------------------------------

proc Goto_Msg_No_Del { msg_number } {
    global message_graphic msg_count current_highlight

    if ($msg_number<$msg_count) {
	set item $message_graphic($msg_number)
	if ($item>=0) {
	    set current_highlight $item
	    Basic_Highlight $current_highlight
	}
	Listbox_Select $msg_number
    }
}

#-----------------------------------------------------------------------
# Center_In_Canvas
#-----------------------------------------------------------------------

proc Center_In_Canvas { graphic } {
    global waiting_history place Scale_Factor

    # determine the left and right positions which frame the activity
    set left [expr ([lindex [$place bbox $graphic] 0])/$Scale_Factor]
    set right [expr ([lindex [$place bbox $graphic] 2])/$Scale_Factor]

    set view_left [Canvas_Left_Edge]
    set view_right [Canvas_Right_Edge]

    if (($left<$view_left)&&($right<$view_left)) {
	Canvas_Center_Around [expr ($left+$right)/2]
    }

    if (($left>$view_right)&&($right>$view_right)) {
	Canvas_Center_Around [expr ($left+$right)/2]
    }
}

###########################################################################
######################## Activity Highlighting ############################

#-----------------------------------------------------------------------
# Basic_Highlight
#-----------------------------------------------------------------------

proc Basic_Highlight { graphic } {
    global waiting_call part_of_call module_call current_highlight

    set current_highlight $graphic

    .info.top.tippy.unselect configure -state normal

    # if the activity that was just selected is a waiting call
    # (ie. the activity is waiting for another module to respond).
    if ($waiting_call($graphic)>0) {
	Show_Message_Structure $graphic
    } else {
	set poc $part_of_call($graphic)
	
	if ($module_call($poc,4)>0) {
	    Show_Message_Structure $module_call($poc,4)
	} else {
	    HighlightQuery \
		    $module_call($poc,0) \
		    $module_call($poc,1) \
		    $module_call($poc,2) \
		    $module_call($poc,3)
	}
    }
}


#-----------------------------------------------------------------------
# Show_Message_Structure
#-----------------------------------------------------------------------

proc Show_Message_Structure { graphic } {
    global waiting_call waiting_history QUERY_MESSAGE

    .info.top.tippy.unselect configure -state normal

    debugp "waiting_call is $waiting_call($graphic)"
    if ($waiting_call($graphic)>0) {
	debugp "waiting_history($graphic,4) is $waiting_history($graphic,4)"
    }

    # Outline the caller module
    OutlineGraphic $graphic
    # Outline the callee module
    OutlineActivity $waiting_history($graphic,5)

    if ($waiting_call($graphic)==$QUERY_MESSAGE) {

	HighlightQueryStruct $graphic

    } elseif ($waiting_call($graphic)>0) {
	# the message type is either an inform, command or broadcast.

	Highlight_ICBM \
		$waiting_history($graphic,0) \
		$waiting_history($graphic,1) \
		$waiting_history($graphic,2) \
		$waiting_history($graphic,3)
    }
	
    # check if the message pends (if it does, show pend structure)
    if ($waiting_call($graphic)>0) {
	if ($waiting_history($graphic,4)>0) {
	    debugp "Shwg HighlightPendStructure $waiting_history($graphic,4)"
	    HighlightPendStructure \
		    $waiting_history($graphic,0) \
		    $waiting_history($graphic,4) \
		    $waiting_history($graphic,1) \
		    $waiting_history($graphic,2) \
		    $waiting_history($graphic,3) \
		    $waiting_history($graphic,5)
	}
    }
}


#-----------------------------------------------------------------------
# OutlineGraphic is for creating the outline of an individual communication
# graphic object. A graphic object is a single item on the canvas.
#
#-----------------------------------------------------------------------

proc OutlineGraphic { comm_graphic } {
    global waiting_history y_loc Scale_Factor
    global MODULE_CALL_OUTLINE_COLOR MODULE_HEIGHT

    set start $waiting_history($comm_graphic,0)
    set end $waiting_history($comm_graphic,1)
    set loc $y_loc($waiting_history($comm_graphic,2))
    
    .a.c.canvas create rectangle \
	    [expr $start*$Scale_Factor-1] \
	    [expr $loc-1] \
	    [expr $end*$Scale_Factor+1] \
	    [expr $loc+$MODULE_HEIGHT+1] \
	    -outline $MODULE_CALL_OUTLINE_COLOR \
	    -width 3 \
	    -tag module_call_arrows
}

#-----------------------------------------------------------------------
#
# OutlineActivity is for creating the outline of a segment of activity
# for a particular module. A "segment of activity" begins when a
# module goes from inactive to active, and ends when the module returns
# to the inactive state. This region may include more that one Tk item
# on the canvas.
#
#-----------------------------------------------------------------------

proc OutlineActivity { activity_number } {
    global module_call y_loc Scale_Factor called_ol_count
    global MODULE_CALLED_OUTLINE_COLOR MODULE_HEIGHT

    if ($activity_number>=0) {
	# the starting X coordinate:
	set start $module_call($activity_number,0)
	# the ending X coordinate:
	set end $module_call($activity_number,1)
	# the Y position of the module:
	set loc $y_loc($module_call($activity_number,2))
	
	.a.c.canvas create rectangle \
		[expr $start*$Scale_Factor-1] \
		[expr $loc-1] \
		[expr $end*$Scale_Factor+1] \
		[expr $loc+1+$MODULE_HEIGHT] \
		-outline $MODULE_CALLED_OUTLINE_COLOR \
		-width 3 \
		-tags "module_call_arrows"
    }
}

#-----------------------------------------------------------------------
# Highlight_ICBM
#
# Highlight call structure for an Inform/Command/Broadcast Message (ICBM)
#-----------------------------------------------------------------------

proc Highlight_ICBM {start end module started_by} {
    global y_loc Scale_Factor module_position
    global MODULE_CALL_OUTLINE_COLOR MODULE_HEIGHT

    # Determine if the source module is positioned above or below
    # the destination module
    if ($module_position($module)<$module_position($started_by)) {
	# destination above, source below
	.a.c.canvas create line \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($module)+$MODULE_HEIGHT+1] \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($started_by)-1] \
		-arrow last \
		-tag module_call_arrows
    } elseif ($module!=$started_by) {
	# source above, destination below
	.a.c.canvas create line \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($module)-1] \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($started_by)+$MODULE_HEIGHT+1] \
		-arrow last \
		-tag module_call_arrows
    } else {

	# this message is sent from and to the same module
	.a.c.canvas create line \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($module)-1] \
		[expr $start*$Scale_Factor-10] \
		[expr $y_loc($module)-20] \
		-tag module_call_arrows
	.a.c.canvas create line \
		[expr $start*$Scale_Factor-10] \
		[expr $y_loc($module)-20] \
		[expr $start*$Scale_Factor+10] \
		[expr $y_loc($module)-20] \
		-tag module_call_arrows
	.a.c.canvas create line \
		[expr $start*$Scale_Factor+10] \
		[expr $y_loc($module)-20] \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($module)-1] \
		-arrow last \
		-tag module_call_arrows
    }
}

#-----------------------------------------------------------------------
# HighlightQuery
#-----------------------------------------------------------------------

proc HighlightQuery {start end module started_by} {
    global y_loc Scale_Factor module_position
    global MODULE_CALL_OUTLINE_COLOR MODULE_HEIGHT

    # Given a communication graphic highlight the overall activity
    # that's occurring for the module that the graphic is a part of

    if ($started_by==-1) {
	# the module is not invoked by another module
	# (when does this occur???)
	.a.c.canvas create rectangle \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($module)-1] \
		[expr $end*$Scale_Factor+1] \
		[expr $y_loc($module)+$MODULE_HEIGHT+1] \
		-outline $MODULE_CALL_OUTLINE_COLOR \
		-width 3 \
		-tag module_call_arrows

    } else {

	# the module activity was invoked by a query or call from another
	# module (thus we can highlight the calling and reply structure).

	.a.c.canvas create rectangle \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($module)-1] \
		[expr $end*$Scale_Factor+1] \
		[expr $y_loc($module)+$MODULE_HEIGHT+1] \
		-outline $MODULE_CALL_OUTLINE_COLOR \
		-width 3 \
		-tag module_call_arrows

	.a.c.canvas create rectangle \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($started_by)-1] \
		[expr $end*$Scale_Factor+1] \
		[expr $y_loc($started_by)+$MODULE_HEIGHT+1] \
		-outline $MODULE_CALL_OUTLINE_COLOR \
		-width 3 \
		-tag module_call_arrows

	if ($module_position($module)<$module_position($started_by)) {
	    .a.c.canvas create line \
		    [expr $start*$Scale_Factor-1] \
		    [expr $y_loc($module)+$MODULE_HEIGHT+1] \
		    [expr $start*$Scale_Factor-1] \
		    [expr $y_loc($started_by)-1] \
		    -arrow first \
		    -tag module_call_arrows
	    .a.c.canvas create line \
		    [expr $end*$Scale_Factor] \
		    [expr $y_loc($module)+$MODULE_HEIGHT+1] \
		    [expr $end*$Scale_Factor] \
		    [expr $y_loc($started_by)-1] \
		    -arrow last \
		    -tag module_call_arrows
	} else {
	    .a.c.canvas create line \
		    [expr $start*$Scale_Factor-1] \
		    [expr $y_loc($module)-1] \
		    [expr $start*$Scale_Factor-1] \
		    [expr $y_loc($started_by)+$MODULE_HEIGHT+1] \
		    -arrow first \
		    -tag module_call_arrows
	    .a.c.canvas create line \
		    [expr $end*$Scale_Factor] \
		    [expr $y_loc($module)-1] \
		    [expr $end*$Scale_Factor] \
		    [expr $y_loc($started_by)+$MODULE_HEIGHT+1] \
		    -arrow last \
		    -tag module_call_arrows
	}
    }
}

#-----------------------------------------------------------------------
# HighlightQueryStruct
#-----------------------------------------------------------------------

proc HighlightQueryStruct { graphic } {
    global y_loc Scale_Factor module_position MODULE_HEIGHT waiting_history

    set start $waiting_history($graphic,0)
    set end $waiting_history($graphic,1)
    set module $waiting_history($graphic,2)
    set started_by $waiting_history($graphic,3)

    # the module activity was invoked by a query or call from another
    # module (thus we can highlight the calling and reply structure).

    if ($module_position($module)<$module_position($started_by)) {
	.a.c.canvas create line \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($module)+$MODULE_HEIGHT+1] \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($started_by)-1] \
		-arrow last \
		-tag module_call_arrows
	.a.c.canvas create line \
		[expr $end*$Scale_Factor] \
		[expr $y_loc($module)+$MODULE_HEIGHT+1] \
		[expr $end*$Scale_Factor] \
		[expr $y_loc($started_by)-1] \
		-arrow first \
		-tag module_call_arrows
    } else {
	.a.c.canvas create line \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($module)-1] \
		[expr $start*$Scale_Factor-1] \
		[expr $y_loc($started_by)+$MODULE_HEIGHT+1] \
		-arrow last \
		-tag module_call_arrows
	.a.c.canvas create line \
		[expr $end*$Scale_Factor] \
		[expr $y_loc($module)-1] \
		[expr $end*$Scale_Factor] \
		[expr $y_loc($started_by)+$MODULE_HEIGHT+1] \
		-arrow first \
		-tag module_call_arrows
    }
}

#-----------------------------------------------------------------------
# HighlightPendStructure
#-----------------------------------------------------------------------

# HighlightPendStructure will draw two lines which show the pending
# structure for a particular message. The lines form a sideways "L"
# with an arrowhead on one end.

proc HighlightPendStructure {query_time handle_time reply_time mod_from \
	mod_to activity} {
    global y_loc Scale_Factor module_position
    global MODULE_CALL_OUTLINE_COLOR MODULE_DISTANCE MODULE_HEIGHT
    global PEND_ARROW_OFFSET

    OutlineActivity $activity

    if ($module_position($mod_to)<$module_position($mod_from)) {
	.a.c.canvas create line \
		[expr $query_time*$Scale_Factor]  \
		[expr $y_loc($mod_to)+$MODULE_DISTANCE-$PEND_ARROW_OFFSET] \
		[expr $handle_time*$Scale_Factor] \
		[expr $y_loc($mod_to)+$MODULE_DISTANCE-$PEND_ARROW_OFFSET] \
		-fill black \
		-tag module_call_arrows

	.a.c.canvas create line \
		[expr $handle_time*$Scale_Factor]  \
		[expr $y_loc($mod_to)+$MODULE_DISTANCE-$PEND_ARROW_OFFSET] \
		[expr $handle_time*$Scale_Factor] \
		[expr $y_loc($mod_to)+$MODULE_HEIGHT+1] \
		-fill black \
		-arrow last \
		-tag module_call_arrows
    } else {
	.a.c.canvas create line \
		[expr $query_time*$Scale_Factor]  \
		[expr $y_loc($mod_to)-12] \
		[expr $handle_time*$Scale_Factor] \
		[expr $y_loc($mod_to)-12] \
		-fill black \
		-tag module_call_arrows

	.a.c.canvas create line \
		[expr $handle_time*$Scale_Factor]  \
		[expr $y_loc($mod_to)-12] \
		[expr $handle_time*$Scale_Factor] \
		[expr $y_loc($mod_to)] \
		-fill black \
		-arrow last \
		-tag module_call_arrows
    }
}


###########################################################################
###########################################################################
# The following section deals with support for allowing user access
# to the incoming data in order to provide additional display objects.

#--- Functions For User ---#

#-----------------------------------------------------------------------
# Create_Event_Line
#
#-----------------------------------------------------------------------

proc Create_Event_Line { label } {
    global place EVENT_LINE_COLOR Scale_Factor time_count

    set x_location $time_count

    set top_alignment [$place create oval \
	    [expr ($x_location-1)*$Scale_Factor-2] 10 \
	    [expr ($x_location-1)*$Scale_Factor+4] 16 -fill white \
	    -outline $EVENT_LINE_COLOR -tags event_line]
    set bottom_alignment [$place create oval \
	    [expr ($x_location-1)*$Scale_Factor-2] 370 \
	    [expr ($x_location-1)*$Scale_Factor+4] 376 -fill white \
	    -outline $EVENT_LINE_COLOR -tags event_line]
    set ln [$place create line [expr ($x_location-1)*$Scale_Factor] 16 \
	    [expr ($x_location-1)*$Scale_Factor] 370 \
	    -fill $EVENT_LINE_COLOR \
	    -tags event_line]
    $place create text [expr ($x_location-1)*$Scale_Factor+17] 5 \
	    -text $label

}

# user.tcl must contain the user's event description code

set USER_HOOK [expr 1 - [ catch { source user.tcl } ]]

##########################################################################
########################### Module Names #################################

#-----------------------------------------------------------------------
# Register_Module
#
#-----------------------------------------------------------------------

proc Register_Module {number} {
    .a.mod.names.label$number configure -bg grey
    .a.mod.names.label$number configure -activebackground grey
}

#-----------------------------------------------------------------------
# Deregister_Module
#
#-----------------------------------------------------------------------

proc Deregister_Module {number} {
    .a.mod.names.label$number configure -bg #cc5555
    .a.mod.names.label$number configure -activebackground #cc5555
    # we may want to completely remove the module from the display
}

#-----------------------------------------------------------------------
# 
# The bindings for the module names have to be done in this manner due
# to some inflexibility in Tcl/Tk. The button widgets are placed
# within a window-item on the "names" canvas. The window-item has a
# number of "features" which make their use more difficult for what's 
# being done here, so there are a few bizarre hacks.
#-----------------------------------------------------------------------

proc SetupModuleNameBindings {number} {
    # these bindinds will all be executed at top-level, so we don't have
    # to declare which variables are global.

    bind .a.mod.names.label$number <Button-1> {
	set curY %y
	set move_distance 0
	# raise the label above the other module labels
	raise %W
    }

    bind .a.mod.names.label$number <B1-Motion> {
	set i $path_of_item(%W)
	.a.mod.names move $i 0 [expr %y-$curY]
	# we have to keep a running count of the total distance that
	# the module has been moved (this variable is move_distance).
	incr move_distance [expr %y-$curY]
	# since %y is in the window, not canvas coordinates
	# we don't update curY.
    }

    bind .a.mod.names.label$number <ButtonRelease-1> {
	# determine to which position the module has just been moved

	# move_d adjusts the moved distance so that you have to move the
	# module name a minumum amount in a particular direction for the
	# module to actually move.
	set move_d [expr $move_distance+($MODULE_HEIGHT+$MODULE_SPACE)/2]
	# delta is the number of position spaces moved
	set delta [expr $move_d / ($MODULE_HEIGHT + $MODULE_SPACE)]
	set i $path_of_item(%W)
	set module $i
	set position $module_position($module)
	set new_position [expr $position+$delta]
	debugp "Mvd: $move_distance Dlta: $delta Mod: $module Pos: $position"

	# we move the module-name graphic back to where it was before
	# the user moved it (it will then be correctly repositioned by
	# Move_Module)
	.a.mod.names move $i 0 [expr -$move_distance]

	# now we actually issue the appropritate Move_Module command
	if ($new_position<1) {
	    Move_Module $module 1
	} elseif ($new_position>$NUMBER_OF_MODULES) {
	    Move_Module $module $NUMBER_OF_MODULES
	} else {
	    Move_Module $module $new_position
	}
    }
}


#-----------------------------------------------------------------------
# 
# New_Module initializes the neccessary data structures for the
# creation of a new module, and produces a graphic position for the
# module. New_Module is only called by the C code, not directly from
# anywhere in the Tcl code.
#
#-----------------------------------------------------------------------

proc New_Module {number label hostname} {
    global current_status INACTIVE_STATE
    global y_loc last_color MODULE_SPACE
    global module_name NUMBER_OF_MODULES module_label module_host
    global previous_status needs_restore pending_count module_position
    global position_holds MODULE_HEIGHT path_of_item
    global MODULE_LABEL_WIDTH MODULE_X_OFFSET MODULE_DISTANCE
    global CANVAS_HEIGHT CANVAS_WIDTH SHOW_HOSTNAME

    incr NUMBER_OF_MODULES

    if ([expr ($NUMBER_OF_MODULES+1)*$MODULE_DISTANCE]>$CANVAS_HEIGHT) {
	set CANVAS_HEIGHT [expr $CANVAS_HEIGHT+$MODULE_DISTANCE]
	set foo [list 0 0 $CANVAS_WIDTH $CANVAS_HEIGHT]
	.a.c.canvas configure -scrollregion $foo
	set foo [list 0 0 100 $CANVAS_HEIGHT]
	.a.mod.names configure -scrollregion $foo
    }

    debugp "New module: $number $label $hostname."
    set module_name($number) $label

    # The y_loc array keeps track of the Y coordinate for the given
    # module number (initially place below all other modules).
    set y_loc($number) [expr $NUMBER_OF_MODULES*$MODULE_DISTANCE]

    if (($SHOW_HOSTNAME)&&([string length $hostname]>0)) { 
	set perloc [expr [string first "." $hostname]-1]
	if ($perloc>0) { \
		set hostname [string range $hostname 0 $perloc] }
	set l "$label\n$hostname"
    } else {
	set l "$label"
    }

    set module_label($number) $label
    set module_host($number) $hostname

    button .a.mod.names.label$number -text $l \
	    -relief raised -pady 10 -border 3

    SetupModuleNameBindings $number

    # the button is displayed on the canvas for module names (this allows
    # for graphically rearranging the order of the module names).
    set item [.a.mod.names create window \
	    $MODULE_X_OFFSET [expr $y_loc($number)+$MODULE_SPACE] \
	    -width $MODULE_LABEL_WIDTH -height $MODULE_HEIGHT \
	    -window .a.mod.names.label$number \
	    -tags "mod_name mod_graphic$number"]

    # check if the width of the text within the button requires a
    # larger width (in which case all buttons will be resized).
    if (0) {
	set $MODULE_LABEL_WIDTH [expr $MODULE_LABEL_WIDTH+5]
	.a.mod.names itemconfigure mod_name -width $MODULE_LABEL_WIDTH
	# we may also need to resize the names canvas
    }

    # given a path name path_of_item returns the canvas item number. 
    # This is needed for selecting the module labels.
    set path_of_item(.a.mod.names.label$number) $item

    # The array current_status holds the state of each module.
    set current_status($number) $INACTIVE_STATE
    set previous_status($number) $INACTIVE_STATE
    set last_color($number) "red"
    set needs_restore($number) 0
    set pending_count($number) 0
    set module_position($number) $NUMBER_OF_MODULES
    # ordering of modules in display
    set position_holds($NUMBER_OF_MODULES) $number
    # every module has a queue of messages that are pending on that
    # module.
    make_queue pend_q$number

    set f "Module_Options $label \$Module_Record($label)"

    .mbar.module.menu add command -label $label \
	    -command $f
}


#-----------------------------------------------------------------------
# 
# Show_Module_Host
# This procedure toggles the displaying of the host (ie. machine) that
# a particular module was run on. The hostname is displayed below the
# module name in the list of modules on the leftmost side of the
# comview window.
#-----------------------------------------------------------------------

proc Show_Module_Host {} {
    global SHOW_HOSTNAME NUMBER_OF_MODULES module_label module_host

    set SHOW_HOSTNAME [expr 1-$SHOW_HOSTNAME]

    for {set loop 1} {($loop <= $NUMBER_OF_MODULES)} {incr loop} {
	if ($SHOW_HOSTNAME) {
	    .a.mod.names.label$loop configure \
		    -text "$module_label($loop)\n$module_host($loop)"
	} else {
	    .a.mod.names.label$loop configure \
		    -text "$module_label($loop)"
	}
    }
}

##########################################################################
############ Repositioning Module Communication and Names ################

#-----------------------------------------------------------------------
# 
# Move_Module takes as parameters an internal module number and a
# number which is the new position of that module (where 1 is the
# uppermost position for module names in the display).
# 
#-----------------------------------------------------------------------

proc Move_Module {module new_location} {
    global place module_position position_holds y_loc current_highlight
    global MODULE_DISTANCE
    
    # delete call arrows (will redraw after module is moved)
    Unselect_Current

    if ($module_position($module)==$new_location) {
	debugp "* already there *"
    } else {
 
	# determine whether this is an upward or downward move
	if ($module_position($module)<$new_location) { 
	    set i 1
	} else {
	    set i -1
	}

	set start $module_position($module)
	set end $new_location

	for {set loop $start} {($loop != $end)} {incr loop $i} {
	    # move this graphic one position
	    $place move activity$position_holds([expr $loop+$i]) \
		    0 [expr -$MODULE_DISTANCE*$i]
	    .a.mod.names move mod_graphic$position_holds([expr $loop+$i]) \
		    0 [expr -$MODULE_DISTANCE*$i]
	    # change it's y coordinate for future use
	    incr y_loc($position_holds([expr $loop+$i])) \
		    [expr -$MODULE_DISTANCE*$i]
	    
	    # change it's location 
	    set module_position($position_holds([expr $loop+$i])) $loop
	    set position_holds($loop) $position_holds([expr $loop+$i])
	}

	set module_position($position_holds($loop)) [expr $loop-$i]

	$place move activity$module 0 \
		[expr $MODULE_DISTANCE*$new_location - $y_loc($module)]
	.a.mod.names move mod_graphic$module 0 \
		[expr $MODULE_DISTANCE*$new_location - $y_loc($module)]

	set module_position($module) $new_location
	set position_holds($new_location) $module

	set y_loc($module) [expr $MODULE_DISTANCE*$new_location]
    }
    
    # re-highlight any activity which was highlighted before the move
    if ($current_highlight>0) { Basic_Highlight $current_highlight }
}

##########################################################################
#################### Message and Module History ##########################


#-----------------------------------------------------------------------
# Message_History
# 
# The Message_History procedure is called to log a particular
# communication activity involving a given message. This maintains a
# global history of (message name ==> list of messages which are
# sending that message).
#-----------------------------------------------------------------------

proc Message_History { message_name msg_count } {
    global Message_Record

    # Check if the given message_name is already in the Message_Record
    # data structure.
    set result [catch { set record $Message_Record($message_name) } ]
    
    if ($result==0) {
	# message_name is already in Message_Record array
	lappend Message_Record($message_name) $msg_count
    } else {
	# first time we've seen this message

	# initialize the value for this particular message
	set Message_Record($message_name) [list $msg_count]

	# add the message name to the "Message" menu. When this
	# message is selected from the menu call Message_Options with
	# the appropriate parameters.

	set f "Message_Options $message_name \$Message_Record($message_name)"

	.mbar.message.menu add checkbutton -label $message_name \
		-command $f
    }
}

#-----------------------------------------------------------------------
# Module_History
# 
#-----------------------------------------------------------------------

proc Module_History { name msg_count } {
    global Module_Record

    set result [catch { set record $Module_Record($name) } ]
    
    if ($result==0) {
	# message_name is already in Message_Record array
	lappend Module_Record($name) $msg_count
    } else {

	# initialize the value for this particular message
	set Module_Record($name) [list $msg_count]

	# we also need to add it to the list of message names
	# available to select from the menu
    }
}

#-----------------------------------------------------------------------
# 
# Message_Options is the function to deal with a selection from the
# "messages" menu.
# 
#-----------------------------------------------------------------------

proc Message_Options { msg_name message_numbers } {
    global lbox_array MESSAGE_LISTBOX_HEIGHT

    # Toggle back the state of the menu "button"
    set i [.mbar.message.menu index $msg_name]
    set hold_command [.mbar.message.menu entrycget $i -command]
    .mbar.message.menu entryconfigure $i -command ""
    .mbar.message.menu invoke $i
    .mbar.message.menu entryconfigure $i -command $hold_command

    # create a new (top-level) window for this particular message
    toplevel .msg$msg_name
    wm title .msg$msg_name "$msg_name messages"

    set ysc ".msg$msg_name.u.sb set"

    frame .msg$msg_name.u

    listbox .msg$msg_name.u.listbox -height $MESSAGE_LISTBOX_HEIGHT \
	    -width 100 -yscrollcommand $ysc

    set sc ".msg$msg_name.u.listbox yview"

    scrollbar .msg$msg_name.u.sb -orient vertical \
	    -command $sc

    set l [llength $message_numbers]

    for {set loop 0} {$loop<$l} {incr loop} {
	.msg$msg_name.u.listbox insert end \
		[.lbo.lbox get $lbox_array([lindex $message_numbers $loop])]
    }

    pack .msg$msg_name.u -side top
    pack .msg$msg_name.u.listbox -side right
    pack .msg$msg_name.u.sb -side left -fill y

    # Create bindings for the listbox so that when an element is
    # clicked it will highlight the corresponding activity in the
    # canvas widget.
    Create_Msg_Window_Binding $msg_name

    frame .msg$msg_name.controls
    
    set kill_me "destroy .msg$msg_name"

    button .msg$msg_name.controls.close -text "Close" \
	    -command $kill_me

    label .msg$msg_name.controls.count \
	    -text "[llength $message_numbers] occurrences"
    
    button .msg$msg_name.controls.all -text "Show All" \
	    -command "Show_All_Message $msg_name"

    set ignore_relief \
	    [expr {[ignored_message_p $msg_name] ? "sunken" : "raised" }]

    button .msg$msg_name.controls.ignore -text "Ignore Message" \
	    -command "ignore_action $msg_name" \
	    -relief $ignore_relief

    pack .msg$msg_name.controls.close -side left -padx 20
    pack .msg$msg_name.controls.count -side left -padx 20
    pack .msg$msg_name.controls.ignore -side right -padx 20
    pack .msg$msg_name.controls.all -side right -padx 20
    pack .msg$msg_name.controls -side bottom
}

proc ignore_action { msg_name } {

    # Update the "Ignore" button to reflect the current state
    if ([ignored_message_p $msg_name]==0) {
	.msg$msg_name.controls.ignore configure -relief sunken
	ignore_message $msg_name
    } else {
	.msg$msg_name.controls.ignore configure -relief raised
	clear_ignore_message $msg_name
    }

    # and change the state of the menu "button" to show ignore status
    set i [.mbar.message.menu index $msg_name]
    set hold_command [.mbar.message.menu entrycget $i -command]
    .mbar.message.menu entryconfigure $i -command ""
    .mbar.message.menu invoke $i
    .mbar.message.menu entryconfigure $i -command $hold_command

}

#-----------------------------------------------------------------------
# 
# Module_Options is the function to deal with a selection from the
# "module" menu.
# 
#-----------------------------------------------------------------------

proc Module_Options { msg_name message_numbers } {
    global lbox_array

    catch {

    # create a new (top-level) window for this particular message
    toplevel .msg$msg_name
    wm title .msg$msg_name "$msg_name Module Messages"

    set ysc ".msg$msg_name.u.sb set"

    frame .msg$msg_name.u

    listbox .msg$msg_name.u.listbox -height 12 -width 100 \
	    -yscrollcommand $ysc

    set sc ".msg$msg_name.u.listbox yview"

    scrollbar .msg$msg_name.u.sb -orient vertical \
	    -command $sc

    set l [llength $message_numbers]

    for {set loop 0} {$loop<$l} {incr loop} {
	.msg$msg_name.u.listbox insert end \
		[.lbo.lbox get $lbox_array([lindex $message_numbers $loop])]
    }

    pack .msg$msg_name.u -side top
    pack .msg$msg_name.u.listbox -side right
    pack .msg$msg_name.u.sb -side left -fill y

    # Create bindings for the listbox so that when an element is
    # clicked it will highlight the corresponding activity in the
    # canvas widget.
    Create_Msg_Window_Binding $msg_name

    frame .msg$msg_name.controls
    
    set kill_me "destroy .msg$msg_name"

    label .msg$msg_name.controls.count \
	    -text "[llength $message_numbers] occurrences"
    
    button .msg$msg_name.controls.close -text "Close" \
	    -command $kill_me

    # is there a better way to do this????
    set sa_command "Show_All_Module $msg_name"

    button .msg$msg_name.controls.all -text "Show All" \
	    -command $sa_command

    pack .msg$msg_name.controls.close -side left -padx 20
    pack .msg$msg_name.controls.count -side left -padx 20
    pack .msg$msg_name.controls.all -side right -padx 20
    pack .msg$msg_name.controls -side bottom
    }
}


#-----------------------------------------------------------------------
# 
# Show_All_Message
# 
#-----------------------------------------------------------------------

# needs to deal with zoom

proc Show_All_Message { message_name } {
    global Message_Record CurrentShowAll

    set message_numbers $Message_Record($message_name)

    set l [llength $message_numbers]

    Unselect_Current
    set CurrentShowAll $message_name

    for {set loop 0} {$loop<$l} {incr loop} {
	Goto_Msg_No_Del [lindex $message_numbers $loop]
    }
}

#-----------------------------------------------------------------------
# 
# Show_All_Message
# 
#-----------------------------------------------------------------------

# needs to deal with zoom

proc Show_All_Module { module_name } {
    global Module_Record CurrentShowAll

    set module_numbers $Module_Record($module_name)

    set l [llength $module_numbers]

    Unselect_Current

    for {set loop 0} {$loop<$l} {incr loop} {
	Goto_Msg_No_Del [lindex $module_numbers $loop]
    }
}

#-----------------------------------------------------------------------
# Create_Msg_Window_Binding
# 
#-----------------------------------------------------------------------

proc Create_Msg_Window_Binding { msg_name } {
    
    bind .msg$msg_name.u.listbox <Button-1> {
	# Grab the text currently selected by X (hopefully within 
	# the listbox)
	set selection_result [catch { set lbox_text [selection get] }]
	# Check that there is a selection made (non-zero means error)
	if ($selection_result==0) {
	    # Find the first colon, the text before it is the message number
	    set loc [expr [string first ":" $lbox_text]-1]
	    # Check that we found a colon
	    if ($loc>=0) {
		# get the text from beginning of line up to colon (this
		# should be the message number)
		set msg_number [string range $lbox_text 0 $loc]
		# we may catch two possible errors:
		catch {
		    # this first conditional statement could generate an
		    # error if msg_number is not actually a number
		    if ($msg_number<$msg_count) {
			Goto_Msg $msg_number
			if (($RunOK==0)||($follow_production==0)) {
			    # we only want to recenter the canvas around
			    # the newly selected item if we are not
			    # currently in Run mode or scroll lock is off.
			    Center_In_Canvas $current_highlight
			}
		    }
		}
	    }
	}
    }
}

##########################################################################
##########################################################################

# The means by which the communication graphics are created and
# maintained is as follows:
#
# Initially two graphics are created for any type of message (query,
# broadcast, command or inform) a graphic at the source module and a
# graphic at the destination module. The width of the destination
# graphic will always be 1/100 of a second. The width of the source
# graphic will be 1/100 of a second for blocking messages and 1/1000
# of a second for non-blocking messages.
#
# The graphics are only updated when a new message arrives. The update
# rules are divided into two categories; an update for a module which
# was not involved in the current communication activity, and an
# update for the two modules which constituted the current
# communication (ie. the source and destination modules).

#-----------------------------------------------------------------------
# updateOthers
#
# Every step in the log file is going to effect only one or two
# modules, and no more. Since each step indicates forward progress of
# time we have to update the graphical widgets indicating activity for
# all the other modules (not involved in the particular message) if
# they are currently active. This means extending the activity graphic
# further to the right to keep up with the "current" time.
# 
#-----------------------------------------------------------------------

proc updateOthers {time_count from_module to_module} {
    global NUMBER_OF_MODULES current_start comm_graphic current_status
    global place y_loc WAITING_STATE waiting_history ACTIVE_STATE
    global MODULE_HEIGHT needs_restore rcv_color INACTIVE_STATE
    global call_forefront module_call Scale_Factor pending_graphic
    global pending_count PENDING_MODULE_HEIGHT pending_start
    global msg_col msg_history waiting_call previous_status

    debugp "in updateOthers $time_count $from_module $to_module"

    # update each communication graphic
    for {set loop 1} {$loop<=$NUMBER_OF_MODULES} {incr loop} {

	# if the module has messages pending then we have to update the
	# pending graphic
	if ($pending_count($loop)>0) {
	    debugp "module $loop has pending message"
	    $place coords $pending_graphic($loop) \
		    [expr $pending_start($loop)*$Scale_Factor] \
		    [expr $y_loc($loop)-$PENDING_MODULE_HEIGHT] \
		    [expr $time_count*$Scale_Factor] \
		    [expr $y_loc($loop)-1]
	}

	set state_ok [expr $current_status($loop)!=$INACTIVE_STATE]

	debugp "needs_restore($loop) is $needs_restore($loop)"
	debugp "state_ok is $state_ok"

	if ($needs_restore($loop)) {
	    # restoration is needed when a module sends a non-blocking
	    # message (ie.inform/command/broadcast/goal) while in 
	    # active mode. It is restored to the active mode after 
	    # transmission of the non-blocking message. needs_restore
	    # will hold the active state of the module before the non-
	    # blocking message was sent.

	    set col $rcv_color($needs_restore($loop))

	    set restore_graphic \
		    [$place create rectangle \
		    [expr ($time_count)*$Scale_Factor] \
		    $y_loc($loop) \
		    [expr ($time_count+1)*$Scale_Factor] \
		    [expr $y_loc($loop)+$MODULE_HEIGHT] \
		    -fill $col \
		    -outline $col -tags single_node \
		    -tags "single_node activity$loop node$col rcv$needs_restore($loop)"]

	    set comm_graphic($loop) $restore_graphic

	    set msg_col($restore_graphic) \
		    "\$rcv_color($needs_restore($loop))"

	    set msg_history($restore_graphic) 0

	    # a module cannot send an inform/command/broadcast while
	    # it is waiting for another module, therefore this cannot
	    # be a waiting call.
	    set waiting_call($restore_graphic) 0

	    # changed this on 6/17/96 -- gmw
	    # set current_status($loop) $ACTIVE_STATE
	    set current_status($loop) $previous_status($loop)

	    set needs_restore($loop) 0

	    set mc [LogActivity $loop 0 $time_count $ACTIVE_STATE 0 \
		    $restore_graphic]

	    set waiting_history($restore_graphic,0) $time_count
	    set waiting_history($restore_graphic,1) [expr $time_count+1]
	    set waiting_history($restore_graphic,2) $from_module
	    set waiting_history($restore_graphic,3) $to_module
	    set waiting_history($restore_graphic,4) -2
	    set waiting_history($restore_graphic,5) $mc


	} elseif (($loop!=$from_module)&&($loop!=$to_module)&&$state_ok) {

	    # an update only occurs for the modules which are not involved
	    # in the current communication activity (these are the two
	    # modules from_module and to_module).

	    # simply resize the current communication graphic
	    # (unless the state is 0, in which case there's 
	    # no graphic) 

	    debugp "current_start:$current_start($loop), $time_count"

	    $place coords $comm_graphic($loop) \
		    [expr $current_start($loop)*$Scale_Factor] \
		    $y_loc($loop) \
		    [expr $time_count*$Scale_Factor] \
		    [expr $y_loc($loop)+$MODULE_HEIGHT]
	    
	    set module_call($call_forefront($loop),1) $time_count
	    
	    # When a module has sent a query it goes into the waiting
	    # state (ie. waiting for a reply). If a module is in the
	    # WAITING_STATE then an update should reflect the new size
	    # of the graphic.
	    if ($current_status($loop)==$WAITING_STATE) {
		set waiting_history($comm_graphic($loop),1) $time_count
	    }
	}
    }
    debugp "TCL leaving Updateothers"
}

#-----------------------------------------------------------------------
# LogActivity
# 
#-----------------------------------------------------------------------

proc LogActivity {module type start_time previous_status started_by \
	call_graphic } {
    global comm_graphic call_count module_call call_forefront 
    global part_of_call INACTIVE_STATE current_start

    if ($previous_status==$INACTIVE_STATE) {
	
	debugp "TCL: creating new activity in LogActivity"

	# we're creating a new communication activity for the module
	incr call_count
	
	set this_item_call_value $call_count
	
	# the module_call array holds the following values:
	# 0: the start of the module activity
	# 1: the end of the module activity
	# 2: the module number
	# 3: the module which "called" this module

	set module_call($this_item_call_value,0) $start_time
	set module_call($this_item_call_value,1) [expr $start_time+1]
 	set module_call($this_item_call_value,2) $module
	set module_call($this_item_call_value,3) $started_by
	set module_call($this_item_call_value,4) $call_graphic
	set call_forefront($module) $this_item_call_value

    } else {
	debugp "TCL: LogActivity updating $module"

	set this_item_call_value $call_forefront($module)
	set module_call($call_forefront($module),1) [expr $start_time]
    }

    set current_start($module) $start_time
    set part_of_call($comm_graphic($module)) $this_item_call_value
    
    return $this_item_call_value
}

#-----------------------------------------------------------------------
# non_blockingp
#
# non_blockingp is a predicate that determines whether the given 
# message type is non-blocking. The following types of TCA messages
# are non-blocking:
#       Inform, Command, Broadcast and Goal messages
#-----------------------------------------------------------------------

proc non_blockingp { message_type } {
    global INFORM_MESSAGE COMMAND_MESSAGE BROADCAST_MESSAGE GOAL_MESSAGE

    if (($message_type==$INFORM_MESSAGE)||($message_type==$COMMAND_MESSAGE)||($message_type==$BROADCAST_MESSAGE)||($message_type==$GOAL_MESSAGE)) { return 1 } else { return 0 }
}

proc Logged_Message_Type { message_type } {
    global QUERY_MESSAGE INFORM_MESSAGE COMMAND_MESSAGE BROADCAST_MESSAGE
    global GOAL_MESSAGE

    if (($message_type==$QUERY_MESSAGE)||($message_type==$INFORM_MESSAGE)||($message_type==$BROADCAST_MESSAGE)||($message_type==$COMMAND_MESSAGE)||($message_type==$GOAL_MESSAGE)) { return 1 } else { return 0 }
    
}

#-----------------------------------------------------------------------
# Activity
# 
#-----------------------------------------------------------------------

proc Activity {from_module to_module message_type message_status name \
	dt from_resourcep} {
    global follow_production msg_count y_loc current_status
    global WAITING_STATE INACTIVE_STATE last_color task_color
    global keep_update NUMBER_OF_MODULES BROADCAST_MESSAGE
    global place Scale_Factor comm_graphic rcv_color
    global ACTIVE_STATE msg_history module_name
    global time_count msg_col call_count waiting_call waiting_history
    global QUERY_MESSAGE GOAL_MESSAGE COMMAND_MESSAGE INFORM_MESSAGE 
    global EXCEPTION_MESSAGE KILLED_MESSAGE BROADCAST_MESSAGE
    global REPLY_MESSAGE needs_restore CANVAS_WIDTH FOLLOW_OFFSET
    global previous_status MODULE_HEIGHT PENDING_MODULE_HEIGHT
    global PENDING_STATUS pending_start pending_count pending_graphic
    global SUCCESS_MESSAGE FAILURE_MESSAGE pend_source
    global Time_Scale_Factor message_graphic PEND_BAR_COLOR

    debugp "Time count: $dt"
    debugp "In TCL procedure Activity (resourcep $from_resourcep)"

    incr time_count [expr $dt-1]
    updateOthers $time_count -1 -1

    # scroll the canvas to keep up with the latest created graphic (if
    # scroll lock is on)
    if ($follow_production==1) {
	Canvas_Moveto \
		[expr (($time_count*$Scale_Factor) - $FOLLOW_OFFSET) / $CANVAS_WIDTH]
    }

    # ****************** Source Module **********************
    
    if ($from_resourcep) {
	debugp "TCL: this is a message from a resource"
	# this is a message from a resource, therefore the "source" module
	# doesn't have a graphic created, because it already has been
	# created.
	
	set cg [dequeue pend_q$from_module]
	set from_graphic $cg
	
	debugp "cg is $cg, time_count=$time_count"

	set waiting_history($cg,4) $time_count

	set pending_count($from_module) \
		[expr $pending_count($from_module)-1]
	
    } elseif ($message_type==$SUCCESS_MESSAGE) {
	debugp "SUCCESS MESSAGE"
	set current_status($from_module) $INACTIVE_STATE
	debugp "msg_count $msg_count, from_module $from_module"
	debugp "comm_graphic $comm_graphic($from_module)"
	set message_graphic($msg_count) $comm_graphic($from_module)
	set msg_history($comm_graphic($from_module)) $msg_count
	.a.mod.names.label$from_module configure -bg grey
	.a.mod.names.label$from_module configure -activebackground grey

    } elseif ($message_type==$FAILURE_MESSAGE) {
	debugp "FAILURE MESSAGE"
	set current_status($from_module) $INACTIVE_STATE
	debugp "msg_count $msg_count, from_module $from_module"
	debugp "comm_graphic $comm_graphic($from_module)"
	set message_graphic($msg_count) $comm_graphic($from_module)
	set msg_history($comm_graphic($from_module)) $msg_count
	.a.mod.names.label$from_module configure -bg grey
	.a.mod.names.label$from_module configure -activebackground grey

    } else {

	set previous_status($from_module) $current_status($from_module)

	if ($message_type==$REPLY_MESSAGE) {
	    # if a module sends a REPLY then it must go into
	    # INACTIVE_STATE after doing so.
	    set current_status($from_module) $INACTIVE_STATE
	    set waiting 0
	} else {
	    # the message is an initiating message (broadcast, query...)
	    set waiting $message_type
	    # the +100 indicates that the module is waiting
	    set current_status($from_module) [expr $message_type+100]
	}

	if [non_blockingp $message_type] {
	    set needs_restore($from_module) $previous_status($from_module)
	    # shouldn't this be previous_status instead of INACTIVE???
	    # It's INACTIVE because of the way non-blocking messages
	    # are drawn (and managed) (doesn't answer above question).
	    set current_status($from_module) $INACTIVE_STATE
	}

	if ($message_type==$QUERY_MESSAGE) {
	    set waiting 1
	    set needs_restore($from_module) 0
	    if ($previous_status($from_module)>0) {
		set last_color($from_module) \
			$rcv_color($previous_status($from_module))
	    }
	}

	set col $task_color($message_type)

	# change color of source module label to indicate new state
	.a.mod.names.label$from_module configure -bg $col
	.a.mod.names.label$from_module configure -activebackground $col

	# set o_col "black"
	set o_col $col

	if ($message_type!=$REPLY_MESSAGE) {
	    debugp "creating source communication graphic"
	    set comm_graphic($from_module) \
		    [$place create rectangle \
		    [expr $time_count*$Scale_Factor] \
		    $y_loc($from_module) \
		    [expr ($time_count+1)*$Scale_Factor] \
		    [expr $y_loc($from_module)+$MODULE_HEIGHT] \
		    -fill $col \
		    -outline $o_col \
		    -tags "single_node activity$from_module node$col src$message_type"]

	}

	set r [catch { set from_graphic $comm_graphic($from_module) }]

	if ($r!=0) {
	    # this probably means correct logging options weren't used
	    puts "comview error: no created source widget"
	    # create_new_source $from_module
	}

	LogActivity $from_module $message_type \
		$time_count $previous_status($from_module) \
		-1 -1

	# this must go here...
	if ($message_type!=$REPLY_MESSAGE) {
	    set msg_col($from_graphic) "\$task_color($message_type)"
	}

	set waiting_call($from_graphic) $waiting
	set last_message_type($from_module) $message_type
	set msg_history($from_graphic) $msg_count

	# Ideally, this should be in LogActivity type function:
	if [Logged_Message_Type $message_type] {
	    set waiting_history($from_graphic,0) $time_count
	    set waiting_history($from_graphic,1) [expr $time_count+1]
	    set waiting_history($from_graphic,2) $from_module
	    set waiting_history($from_graphic,3) $to_module
	    set waiting_history($from_graphic,4) -1
	    set waiting_history($from_graphic,5) -1
	}
    }

    debugp "message type is $message_type (reply=$REPLY_MESSAGE)"
    
    # this is neccessary for the listbox so that we can highlight the
    # graphical activity for a particular message number. it builds an 
    # array of (message number) --> (canvas graphic), for every message.

    if ($message_type==$REPLY_MESSAGE) {
	set message_graphic($msg_count) $comm_graphic($to_module)
	debugp "storing message_graphic($msg_count) $comm_graphic($to_module)"
    } elseif (($message_type!=$SUCCESS_MESSAGE)&&($message_type!=$FAILURE_MESSAGE)) {
	set message_graphic($msg_count) $from_graphic
	debugp "storing message_graphic($msg_count) $from_graphic"
    }

    # ****************** Destination Module **********************

    debugp "working on destination module (to=$to_module)"

    if ($message_status==$PENDING_STATUS) {
	# message is from a module to a resource module (ie. a message
	# that pends).

	debugp "TCL: pending message"

	set col $PEND_BAR_COLOR
	set o_col $col

	if ($pending_count($to_module)==0) {
	    # only create a pending graphic if there isn't already
	    # one for the module
	    set pending_graphic($to_module) [$place create rectangle \
		    [expr $time_count*$Scale_Factor] \
		    [expr $y_loc($to_module)-$PENDING_MODULE_HEIGHT] \
		    [expr ($time_count+1)*$Scale_Factor] \
		    [expr $y_loc($to_module)-1] \
		    -fill $col \
		    -outline $o_col \
		    -tags "pending_node activity$to_module"]

	    set pend_source($pending_graphic($to_module)) \
		    $from_graphic

	    set pending_start($to_module) $time_count
	}

	set pending_count($to_module) [expr $pending_count($to_module)+1]
	
	# this keeps track of which graphic the particular pending event
	# belongs to (ie. calling module)
	enqueue pend_q$to_module $from_graphic
	
    } elseif ($message_type==$SUCCESS_MESSAGE) {
    } elseif ($message_type==$FAILURE_MESSAGE) {
    } else {

	if ($message_type==$REPLY_MESSAGE) {
	    set col $last_color($to_module)

	    debugp "setting current_status($to_module) $previous_status($to_module)"

	    set current_status($to_module) $previous_status($to_module)

	    # change color of module labels to indicate new state
	    .a.mod.names.label$from_module configure -bg grey
	    .a.mod.names.label$from_module configure -activebackground grey
	    .a.mod.names.label$to_module configure -bg grey	
	    .a.mod.names.label$to_module configure -activebackground grey
	} else {
	    set col $rcv_color($message_type)
	}

	if ($message_type==$QUERY_MESSAGE) {
	    set last_color($to_module) $col
	}
	
	set prev_state $previous_status($to_module)
	set previous_status($to_module) $current_status($to_module)

	# There are two conditions under which we create a new graphic item:
	# (1) Message being sent is a Query/Inform/Command/Broadcast/Goal
	#     (ie. not a reply message)
	# (2) Module is being sent a reply message, but it sent the
	#     query while active, thus it should have a graphic
	#     created that reflects the state of the module at the
	#     time that the query was sent.
	
	if (($message_type!=$REPLY_MESSAGE)||($prev_state!=$INACTIVE_STATE)) {
	    #set o_col "black"
	    set o_col $col

	    debugp "Outline color is $o_col"

	    # change color of destination module label to indicate new state
	    .a.mod.names.label$to_module configure -bg $col
	    .a.mod.names.label$to_module configure -activebackground $col

	    if ($message_type==$REPLY_MESSAGE) {
		set rcv_binding "rcv$QUERY_MESSAGE"
	    } else {
		set rcv_binding "rcv$message_type"
	    }
	    
	    set to_graphic [$place create rectangle \
		    [expr $time_count*$Scale_Factor] \
		    $y_loc($to_module) \
		    [expr ($time_count+1)*$Scale_Factor] \
		    [expr $y_loc($to_module)+$MODULE_HEIGHT] \
		    -fill $col \
		    -outline $o_col \
		    -tags "single_node activity$to_module node$col $rcv_binding"]

	    debugp "to_graphic is $to_graphic"

	    if ($message_type==$REPLY_MESSAGE) {
		set current_status($to_module) $previous_status($to_module)
		set msg_col($to_graphic) "\$rcv_color($QUERY_MESSAGE)"
	    } else {
		set current_status($to_module) $message_type
		set msg_col($to_graphic) "\$rcv_color($message_type)"
	    }

	    set comm_graphic($to_module) $to_graphic
	    set msg_history($to_graphic) $msg_count
	    set waiting_call($to_graphic) 0
	    
	    if ($from_resourcep) {
		# if this message is from a resource (ie. the message
		# was pending, but is no longer) then we determine
		# which module the message was originally from.
		set from_module $waiting_history($cg,2)
	    }

	    # LogActivity is going to return the call item that this
	    # current activity will belong to.
    	    set call_item [LogActivity $to_module $message_type \
		    $time_count $previous_status($to_module) \
		    $from_module $from_graphic]

	    if ($from_resourcep) {
		# the message is from a resource therefore the graphic from
		# which the message originated needs to know the destination
		set waiting_history($cg,5) $call_item

	    } elseif ($message_type!=$REPLY_MESSAGE) {
		# for Query, Command, Inform, Broadcast messages we
		# keep track of the destination activity-record (Reply
		# messages do not result in the creation of a graphic).
		set waiting_history($from_graphic,5) $call_item
	    }

	}
    }

    incr time_count
    incr msg_count

    updateOthers $time_count $from_module $to_module

    grow_canvas [expr $dt*$Scale_Factor]
    
    # force Tk to update the graphics
    if ($keep_update==1) update
    
    debugp "Exiting TCL Activity"
}

# ---------------- end of Activity procedure ---------------------------


#-----------------------------------------------------------------------
# Read_In_Data
#
# Read_In_Data calls the C function that reads a single line from the TCA
# log file and returns the parsed data in a predetermined format. The C
# code will either return a string of the form "list d1 d2 d3 ... dn"
# where d1...dn are the various data elements we want, or "perror"
# (which signifies a parse error or no data). If there is a valid
# message parsed, then this string is evaluated to produce a Tcl list.
# This roundabout manner of returning multiple values is neccessary 
# because some of data elements are strings possibly with whitespace
# characters. The standard Tcl function "scan" is unable to correctly
# parse such strings.

#-----------------------------------------------------------------------

proc Read_In_Data {} {

    # C_Get_TCA_Data is a C function defined as Get_TCA_Data in the 
    # file tcl-framework.c
    set inl [C_Get_TCA_Data]

    if ([string compare $inl "perror"]==0) {
	# inl does not contain an actual message
	set in $inl
    } else {
	# inl does contain an actual message. The format of the
	# message is "list arg1 arg2 ... arg3", thus by evaluating
	# this message we get a list of all the arguments.
	set in [eval $inl]
    }

    debugp "TCL read: $in"
    
    return $in
}

proc valid_message_type { msg_type } {
    global QUERY_MESSAGE GOAL_MESSAGE COMMAND_MESSAGE INFORM_MESSAGE
    global EXCEPTION_MESSAGE KILLED_MESSAGE INTERVAL_MESSAGE POINT_MESSAGE 
    global FIRE_MESSAGE REPLY_MESSAGE FAILURE_MESSAGE SUCCESS_MESSAGE
    global BYPASS_MESSAGE PCONSTRAINT_MESSAGE RETRY_MESSAGE WILL_KILL_MESSAGE
    global TCONSTRAINT_MESSAGE WARNING_MESSAGE CONNECT_MESSAGE
    global DISCONNECT_MESSAGE BROADCAST_MESSAGE


    debugp "valid_message_type \"$msg_type\""
    
    if ([string compare $msg_type ""]==0) { return 0 }
    
    if ($msg_type==$QUERY_MESSAGE||$msg_type==$GOAL_MESSAGE||$msg_type==$COMMAND_MESSAGE||$msg_type==$INFORM_MESSAGE||$msg_type==$EXCEPTION_MESSAGE||$msg_type==$INTERVAL_MESSAGE||$msg_type==$POINT_MESSAGE||$msg_type==$FIRE_MESSAGE||$msg_type==$REPLY_MESSAGE||$msg_type==$FAILURE_MESSAGE||$msg_type==$SUCCESS_MESSAGE||$msg_type==$BYPASS_MESSAGE||$msg_type==$PCONSTRAINT_MESSAGE||$msg_type==$RETRY_MESSAGE||$msg_type==$WILL_KILL_MESSAGE||$msg_type==$WARNING_MESSAGE||$msg_type==$CONNECT_MESSAGE||$msg_type==$DISCONNECT_MESSAGE||$msg_type==$BROADCAST_MESSAGE) {
	return 1 }
	
	return 0
}

#-----------------------------------------------------------------------
# Step
#
# Does the following:
# 1) Read in parsed data
# 2) Handle that message
# 
#-----------------------------------------------------------------------


proc Step {} {
    global module_name SHOW_LISTBOX msg_count
    global time_count lbox_array LOG_IGNORED_MESSAGES
    global CONNECT_MESSAGE DISCONNECT_MESSAGE RunOK USER_HOOK
    global Tview_Running cur current_highlight

    set in [Read_In_Data]

    #updateListbox 
    #update
    #return 1

    debugp "TCL: Read in data"

    if ([string compare $in "perror"]==0) {
	# A parse error can mean one of two things:
	#  (1) reached the end of the log file
	#  (2) the parser really did generate an error
	debugp "TCL: C had a parse error"
	if ($RunOK) { 
	    .info.top.tippy.run configure -bg lightyellow
	    .info.top.tippy.run configure -activebackground #deff10
	}
	return -1
    } elseif ([valid_message_type [lindex $in 4]]==0) {
	return -1
    } else {
	set from_module [lindex $in 0]
	set from_name [lindex $in 1]
	set to_module [lindex $in 2]
	set to_name [lindex $in 3]
	set message_type [lindex $in 4]
	set message_status [lindex $in 5]
	set message_name [lindex $in 6]
	set delta_t [lindex $in 7]
	set from_resourcep [lindex $in 8]

	# Message_History keeps track of all occurrences of
	# any particular message. Ignore module registration/dereg.
	if ($message_type!=19) {
	    Message_History $message_name $msg_count
	}

	# Module_History keeps track of all the messages which
	# involve any particular module.
	Module_History $from_name $msg_count
	# If the message is being sent to and from the same module we
	# don't want to log it twice here.
	if ([string compare $from_name $to_name]!=0) {
	    Module_History $to_name $msg_count
	}

	# the user can specify a settings file which will allow a
	# particular message or messages from a particular module to be
	# ignored. this means that they aren't displayed in the canvas
	# display. optionally you can have the ignored messages still
	# logged to the listbox log window.

	if ([ignored_message_p $message_name]||[ignored_module_p $from_name]||[ignored_module_p $to_name]) {
	    if ($LOG_IGNORED_MESSAGES) { 
		incr msg_count
		updateListbox 
	    }
	    update
	} else {
	    # the message is not ignored

	    updateListbox
	    
	    if ($message_type==$CONNECT_MESSAGE) {
		debugp "Module Connection Message $from_module $to_name"
		Register_Module $from_module
	    } elseif ($message_type==$DISCONNECT_MESSAGE) {
		debugp "Module Disconnection Message $from_module"
		Deregister_Module $from_module
	    } else {
		debugp "TCL delta_t is $delta_t"

		Activity $from_module $to_module $message_type \
			$message_status $message_name $delta_t $from_resourcep

		debugp "TCL from: $from_module to: $to_module dt: $delta_t type: $message_type name: $message_name stat: $message_status"

		if ($USER_HOOK) {
		    User_Event_Hook $from_name $to_name \
			    $message_type $message_name
		}
	    }
	}

	if ($current_highlight>0) {
	    set hold $current_highlight
	    Unselect_Current
	    Basic_Highlight $hold
	}
	
	return 1
	
	debugp ""
    }
}

set Tview_Running [expr 1 - [catch { send tview Ping }]]
if ($Tview_Running) { send comview set Comview_Running 1 }

proc LocalStep {} {
    global Tview_Running

    if ($Tview_Running) { send tview Step }
    return [Step]
}
#-----------------------------------------------------------------------
# StepCommand is the procedure that is executed when the "Run" button is 
# pressed.
#-----------------------------------------------------------------------

proc StepCommand {} {
    global RunOK

    # if we're currently Running this will stop that:
    set RunOK 0
    LocalStep
}

#-----------------------------------------------------------------------
# 
# RunCommand is the procedure that is executed when the "Run" button is 
# pressed.
#
#-----------------------------------------------------------------------

proc RunCommand {} {
    global RunOK
    set RunOK [expr 1-$RunOK]
    if ($RunOK==1) { Run }
}

#-----------------------------------------------------------------------
# 
# Run
#
#-----------------------------------------------------------------------

proc Run {} {
    global RunOK

    # Set the colors of the "Run" button (to greenish colors):
    .info.top.tippy.run configure -bg #11f033
    .info.top.tippy.run configure -activebackground #33ff44
    .info.top.tippy.run configure -relief sunken

    set result 0

    # Loop while it's ok to continue running:
    for {} {($RunOK==1)} {} {
	if ($result==-1) {
	    update
	}
	set result [LocalStep]
    }
    
    # Change the colors of the "Run" button back to the default grey
    # settings (now that run mode is off).
    .info.top.tippy.run configure -bg #d9d9d9
    .info.top.tippy.run configure -activebackground #ececec
    .info.top.tippy.run configure -relief raised
}

#-----------------------------------------------------------------------
# 
# Stepn is only a development/debugging function
#
#-----------------------------------------------------------------------

proc Stepn {n} {
    for {set loop 1} {$loop < $n} { incr loop } {
	Step
	LocalStep

    }
}

# For Send:
proc Ping {} {
    return 1
}

debugp "comview.tcl script loaded ok"

# mouse.tcl
# Canvas Mouse Bindings


# this binding has some unusual semantics because the action described
# here happens AFTER the action that MAY occur for the canvas binding
# for a single node. Thus to get the functionality where you can
# unselect an object by clicking on empty parts of the canvas we have
# to determine whether the click had an earlier binding event (thus
# the variable button_1_object).

set button_1_object 0

bind .a.c.canvas <Button-1> { 
    if ($button_1_object==0) {
	# If there's activity previously highlighted, delete it.
	Unselect_Current
    }

    set button_1_object 0
}

########################################################################

# Whenever the mouse cursor is in the graphical activity window
# calculate the time that corresponds to the current mouse position and
# display it.

bind .a.c.canvas <Motion> {
    # ensure that there's actually been a message logged
    if ($msg_count>1) {
	# Calculate the current X position of mouse in canvas coords
	set offset [expr ($scroll_offset+%x)/$Scale_Factor]
	set time_str [Time_String $offset]
	.info.top.time.tlabel configure -text $time_str

	# mouse_time will display the time coordinate below the mouse
	if ($mouse_time>0) { 
	    .a.c.canvas delete $mouse_time
	    set mouse_time \
		    [.a.c.canvas create text %x [expr %y+20] \
		    -text $time_str]
	}
    }
}

bind .a.c.canvas <Any-Leave> {
    if ($mouse_time>0) { 
	.a.c.canvas delete $mouse_time
    }
}

#---------------------------------------------------------------------
# Third-button Panner (idea stolen from ControlShell's Graphical
# Editors).
#---------------------------------------------------------------------

bind . <Button-3> {
    set testX %x
    set testY %y

    .a.c.canvas configure -xscrollincrement 2
    # .a.c.canvas configure -yscrollincrement 1
    # .a.mod.names configure -yscrollincrement 1
}

bind . <B3-Motion> {
		
    set dtX [expr %x-$testX]
    set dtY [expr %y-$testY]

    .a.c.canvas xview scroll $dtX unit
    Canvas_Change_Time
    # .a.c.canvas yview scroll $dtY unit
    # .a.mod.names yview scroll $dtY unit
    
    set testX %x
    # set testY %y
	
    update
}

bind . <ButtonRelease-3> {
    .a.c.canvas configure -xscrollincrement 0
    # .a.c.canvas configure -yscrollincrement 0
    # .a.mod.names configure -yscrollincrement 0
}


########################################################################

# This allows a button click in the Listbox to highlight the graphical
# activity that corresponds to the clicked message. A binding is
# created for a button press in the listbox and the line which was
# clicked is identified using the [selection get] operation -- this
# gives the text of the line within the listbox. The text before the
# colon is the message number, this is parsed out. The message_graphic
# data structure returns the graphical activity for that message number.

# try changin Button-1 to ButtonRelease-1

bind .lbo.lbox <Button-1> {
    # Grab the text currently selected by X (hopefully within the listbox)
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
		    Unselect_Current
		    set item $message_graphic($msg_number)
		    if ($item>=0) {
			set current_highlight $item
			Basic_Highlight $current_highlight
		    }

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

##########################################################################
############### Communication Graphic BUTTON BINDINGS ####################

# These are mouse event bindings for the rectangular communication
# graphics that appear within the canvas widget. They all have the tag
# "single_node" given to them.

.a.c.canvas bind single_node <Button-1> {

    # remove any selection lines that were previously drawn on the canvas
    Unselect_Current

    set cur [.a.c.canvas find withtag current]
    debugp "button-1 selection: $cur"

    # Highlight the calling structure for this particular element
    Basic_Highlight $cur

    if ($waiting_call($cur)==0) {
	# user may have selected the destination end of an inform,
	# command or broadcast message. If so we want to select a
	# different line in the log file.
	set cur $module_call($part_of_call($cur),4)
    } 

    set current_highlight $cur

    debugp "msg_history for $cur is $msg_history($cur)"

    if ($msg_history($cur)>0) {
	# if there's an associated line in the listbox for this item 
	# (this is tested by ($msg_history($cur)>0)), then highlight it.
	Listbox_Select $msg_history($cur)
    } else {
	# there is no associated line, so make sure there is nothing
	# highlighted
	.lbo.lbox selection clear 0 end
    }

    # This is so that we can deselect using button-1 when we're not
    # actually selecting something (ie. on empty parts of the canvas).
    set button_1_object 1
}

.a.c.canvas bind single_node <Button-3> {

    # remove any selection lines that were previously drawn on the canvas
    Unselect_Current

    set cur [.a.c.canvas find withtag current]
    debugp "button-1 selection: $cur"

    set current_highlight $cur

    debugp "msg_history for $cur is $msg_history($cur)"

    # if there's an associated line in the listbox for this item (this
    # is tested by ($msg_history($cur)>0)), then highlight it.
    if ($msg_history($cur)>0) {
	Listbox_Select $msg_history($cur)
    } else {
	# there is no associated line, so make sure there is nothing
	# highlighted
	.lbo.lbox selection clear 0 end
    }

    # Highlight the calling structure for this particular element
    Basic_Highlight $cur

    # This is so that we can deselect using button-1 when we're not
    # actually selecting something (ie. on empty parts of the canvas).
    set button_1_object 0
}


#.a.c.canvas bind single_node <Button-2> {
#
#    Unselect_Current
#    
#    set poc $part_of_call([.a.c.canvas find withtag current])
#
#    debugp "module_call($poc,4) (the call_graphic) was $module_call($poc,4)"
#
#    if ($module_call($poc,4)>0) {
#	Show_Message_Structure $module_call($poc,4)
#    } else {
#	HighlightQuery \
#		$module_call($poc,0) \
#		$module_call($poc,1) \
#		$module_call($poc,2) \
#		$module_call($poc,3)
#    }
#}


# The third button only has an action for a graphical activity that is
# calling another module (ie. the source end of a Query, Inform,
# Command or Broadcast). It will highlight the time where the source
# module is sending the message and the activity that occurs in the
# destination module before it completes (for a query completion is
# indicated by a reply message. Inform, Command and Broadcast messages
# complete with a Success or Failure message).

#.a.c.canvas bind single_node <Button-3> {
#
#    .info.top.time.tlabel configure \
#	    -text ""
#
#    # If there's activity previously highlighted, delete it.
#    Unselect_Current
#
#    set selected_graphic [.a.c.canvas find withtag current]
#
#    Show_Message_Structure $selected_graphic
#}

# This is the binding for when the mouse cursor moves over any
# communication activity segment. We simply create a black outline
# around the segment over which the mouse currently resides.

.a.c.canvas bind single_node <Any-Enter> {

    .a.c.canvas itemconfigure current -outline black
    .a.c.canvas raise current

    # keep track of the color so we can restore it when cursor exits
    set last_enter_outline_color \
	    $msg_col([.a.c.canvas find withtag current])
}

.a.c.canvas bind single_node <Any-Leave> {
    eval "set oc $last_enter_outline_color"
    .a.c.canvas itemconfigure current -outline $oc
}

# This is the binding for when the mouse cursor moves over any
# communication activity segment. We simply create a black outline
# around the segment over which the mouse currently resides.

.a.c.canvas bind pending_node <Any-Enter> {

    .a.c.canvas itemconfigure current -outline black
    .a.c.canvas raise current
}

.a.c.canvas bind pending_node <Any-Leave> {
    .a.c.canvas itemconfigure current -outline $PEND_BAR_COLOR
}

.a.c.canvas bind pending_node <Button-1> {
    set this [.a.c.canvas find withtag current]
    Unselect_Current
    Basic_Highlight $pend_source($this)
    # This is so that we can deselect using button-1 when we're not
    # actually selecting something (ie. on empty parts of the canvas).
    set button_1_object 1
}

##########################################################################
# Selecting a region

bind .a.c.canvas <Button-2> {
    set region_x1 [expr $scroll_offset+%x]
    set region_y1 [expr %y+[lindex [.a.c.canvas yview] 0]*$CANVAS_HEIGHT]
     
    .a.c.canvas create rectangle \
	    $region_x1 $region_y1 \
	    $region_x1 $region_y1 \
	    -tag "region"
}

bind .a.c.canvas <B2-Motion> {
    set y [expr %y+[lindex [.a.c.canvas yview] 0]*$CANVAS_HEIGHT]
    .a.c.canvas coords region $region_x1 $region_y1 \
	    [expr %x+$scroll_offset] $y
}

bind .a.c.canvas <ButtonRelease-2> {
    set y [expr %y+[lindex [.a.c.canvas yview] 0]*$CANVAS_HEIGHT]

    set reitems [.a.c.canvas find enclosed \
	    $region_x1 $region_y1 [expr %x+$scroll_offset] $y]

    .a.c.canvas delete region

    # highlight each item (if it's a valid single_node)
    foreach i $reitems {
	# check if the item is a valid single_node
	if ([lsearch [.a.c.canvas gettags $i] single_node]>=0) {
	    # highlight the calling structure for this particular element
	    Basic_Highlight $i
	}
    }
}


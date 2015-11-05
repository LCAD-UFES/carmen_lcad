########################################################################
########################### LOG WINDOW #################################

# The listbox is used to display the log data read from TCA.
# The user can select (ie. click) a line in the log window and have the
# call arrows appear where that event occurs within the graphics canvas.

set SHOW_LISTBOX 1

if ($SHOW_LISTBOX) {
    frame .lbo
    canvas .lbo.spacer -width 0 -height 0

    listbox .lbo.lbox -height $LISTBOX_HEIGHT -width $LISTBOX_WIDTH \
	    -yscrollcommand ".lbo.sb set"
    .lbo.lbox selection anchor end

    scrollbar .lbo.sb -orient vertical -command ".lbo.lbox yview"

    pack .lbo
    pack .lbo.spacer -side left -padx 130
    pack .lbo.lbox -side right
    pack .lbo.sb -side left -fill y


}

#-----------------------------------------------------------------------
# Listbox_Select
#-----------------------------------------------------------------------

proc Listbox_Select { msg_num } {
    global lbox_array HALF_LISTBOX_HEIGHT msg_number

    # Remove the current highlight bar within the ListBox and then
    # highlight the line that corresponds to the given message number.

    .lbo.lbox selection clear 0 end

    # msg_number is a global variable that keeps track of the
    # currently selected message.
    set msg_number $msg_num

    set location $lbox_array($msg_num)
    # highlight the new selection
    .lbo.lbox selection set $location
    # center the new selection within the listbox
    .lbo.lbox yview [expr $location-$HALF_LISTBOX_HEIGHT]
}

# the lbox_array takes a msg and a line within the listbox and keeps
# track of the line within the listbox that a particular message
# corresponds to.

#-----------------------------------------------------------------------
# add_to_lbox_array
# This function is only called from C code.
#-----------------------------------------------------------------------
proc add_to_lbox_array { msg_number lbox_number } {
    global lbox_array
    
    debugp "add_to_lbox_array $msg_number $lbox_number"
    set lbox_array($msg_number) $lbox_number
}

#-----------------------------------------------------------------------
# updateListbox
#-----------------------------------------------------------------------
proc updateListbox {} {
    # call the C function to update listbox (does what???)
    C_Update_Listbox
}

#-----------------------------------------------------------------------
# scrollListbox
#-----------------------------------------------------------------------
proc scrollListbox {} {
    global follow_production
    
    # we only actually scroll the listbox if scroll-lock is on (this
    # is the same scroll-lock used by the canvas graphical window).
    if ($follow_production) {
	.lbo.lbox yview scroll 1 units    
    }
}

########################################################################
###################### SEARCHING LOG WINDOW ############################

#-----------------------------------------------------------------------
# search_log_window
#-----------------------------------------------------------------------
proc search_log_window { str from_line } {
    global end Search_Mode

    set end [.lbo.lbox size]

    set not_found_count 0

    for {set loop $from_line} {(($loop < $end)&&($Search_Mode>0))} \
	    {incr loop} {
	set search_result [string first $str [.lbo.lbox get $loop]]
	if ($search_result>=0) { 
	    return $loop
	} else {
	    incr not_found_count
	    if ($not_found_count>50) {
		update
		set not_found_count 0
	    }
	}
    }

    # couldn't find the string in the log window
    return -1
}

#-----------------------------------------------------------------------
# reverse_search_log_window
# search the log window in the reverse direction.
#-----------------------------------------------------------------------
proc reverse_search_log_window { str from_line } {
    global end

    set end [.lbo.lbox size]

    for {set loop $from_line} {($loop >= 0)} {incr loop -1} {
	set search_result [string first $str [.lbo.lbox get $loop]]
	if ($search_result>=0) { return $loop }
    }

    # couldn't find the string in the log window
    return -1
}

#-----------------------------------------------------------------------
# Enter_Goto_Mode is for opening up the mini-buffer and allowing the
# user to enter a specific message number to jump to.
#-----------------------------------------------------------------------
proc Enter_Goto_Mode {} {
    global Goto_Mode minibuf

    if ($Goto_Mode==0) {
	set Goto_Mode 1
	.info.bot.mini.command configure -text "Goto message: "
	.info.bot.mini.buf configure -width 70 -relief sunken -text ""
    }
}

proc Enter_Goto_Time_Mode {} {
    global Goto_Mode minibuf

    if ($Goto_Mode==0) {
	set Goto_Mode 2
	.info.bot.mini.command configure -text "Goto time: "
	.info.bot.mini.buf configure -width 70 -relief sunken -text ""
    }
}

proc End_Goto_Mode {} {
    global Goto_Mode minibuf

    if ($Goto_Mode>0) {
	.info.bot.mini.buf configure -text "" -relief flat
	.info.bot.mini.command configure -text ""
	set Goto_Mode 0
	set minibuf ""
    }
}

proc Enter_Search_Mode {direction} {
    global Search_Mode search_start_loc last_lbox_find started_at
    global minibuf last_search_string search_direction

    if ($Search_Mode==1) {
	if ([string compare $minibuf ""]==0) {
	    set minibuf $last_search_string
	    .info.bot.mini.buf configure -text $minibuf
	}
	set search_start_loc [expr $last_lbox_find+$direction]
	set search_direction $direction
	Search_For_Message
    } else {
	set search_direction $direction
	set Search_Mode 1
	# configure the part of the minibuffer that displays the command
	.info.bot.mini.command configure -text "Search: "
	# the position from which we're going to search is from the currently
	# selected line in the log-file listbox.
	set search_start_loc [.lbo.lbox curselection]
	if ([string compare $search_start_loc ""]==0) {
	    set search_start_loc 0
	}
	set started_at $search_start_loc
	.info.bot.mini.buf configure -width 70 -relief sunken -text ""
    }
}

proc Search_For_Message {} {
    global minibuf last_lbox_find search_start_loc search_direction
    
    if ($search_direction==1) {
	set r [search_log_window $minibuf $search_start_loc]
    } else {
	set r [reverse_search_log_window $minibuf $search_start_loc]
    }

    if ($r==-1) {
	# search failed, blink mini-buffer
	.info.bot.mini.buf configure -bg black
	update
	after 2
	.info.bot.mini.buf configure -bg #d9d9d9
	update
    } else {
	# search was successful
	set last_lbox_find $r
	.lbo.lbox select clear 0 end
	.lbo.lbox see $r
	.lbo.lbox selection set $r
    }
}

proc End_Search_Mode {} {
    global Search_Mode minibuf last_lbox_find search_start_loc
    global last_search_string

    if ($Search_Mode==1) {
	.info.bot.mini.buf configure -text "" -relief flat
	.info.bot.mini.command configure -text ""
	set Search_Mode 0
	set last_search_string $minibuf
	set minibuf ""
	set last_lbox_find 0
	set search_start_loc 0
    }
}



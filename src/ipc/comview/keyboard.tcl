# -------------------------------------------------------------------- #
#
# TCA Communications Graphics Tool
# Keyboard Bindings
#
# -------------------------------------------------------------------- #

bind . <Alt-p> {
    End_Search_Mode
    set msg_number 1
    Goto_Msg 1
}

bind . <Alt-n> {
    End_Search_Mode
    set msg_number [expr $msg_count-1]
    Goto_Msg $msg_number
}

bind . <Control-p> {
    End_Search_Mode
    if ($msg_number>1) {
	set msg_number [expr $msg_number-1]
	Goto_Msg $msg_number
	update
    }
}

bind . <Control-n> {
    End_Search_Mode
    if ($msg_number<($msg_count-1)) {
	set msg_number [expr $msg_number+1]
	Goto_Msg $msg_number
	update
    }
}

bind . <Control-f> {
    if ($new_selection<[expr [.lbo.lbox size]-1]) {
	# Remove the current highlight bar within the ListBox and highlight
	# line that corresponds to the newly selected communication activity.
	.lbo.lbox selection clear 0 end
	set new_selection [expr $new_selection+1]
	# highlight the new selection
	.lbo.lbox selection set $new_selection
	# center the new selection within the listbox
	.lbo.lbox yview [expr $new_selection-$HALF_LISTBOX_HEIGHT]
    }
}

bind . <Control-b> {
    if ($new_selection>0) {
	# Remove the current highlight bar within the ListBox and highlight
	# line that corresponds to the newly selected communication activity.
	.lbo.lbox selection clear 0 end
	set new_selection [expr $new_selection-1]
	# highlight the new selection
	.lbo.lbox selection set $new_selection
	# center the new selection within the listbox
	.lbo.lbox yview [expr $new_selection-$HALF_LISTBOX_HEIGHT]
    }
}

#--------------- Canvas traversal ---------------#


bind . <space> {
    if ($Search_Mode==1) {
	append minibuf " "
	.info.bot.mini.buf configure -text $minibuf

	Search_For_Message
    } else {
	Canvas_Scroll 1 page
    }
}

bind . <Delete> {
    Canvas_Scroll -1 page
}

bind . <BackSpace> {
    if (($Search_Mode==1)||($Goto_Mode>0)) {
	set minibuf [string range $minibuf \
		0 [expr [string length $minibuf]-2]]
	.info.bot.mini.buf configure -text $minibuf

	if ($Search_Mode==1) { 
	    set search_start_loc $started_at
	    Search_For_Message
	}
    } else {
	Canvas_Scroll -1 page
    }
}

bind . <leftarrow> {
    Canvas_Scroll -1 unit
}

bind . <rightarrow> {
    Canvas_Scroll 1 unit
}

bind . <uparrow> {
}

bind . <downarrow> {
}

bind . <Control-u> {
    # If there's activity previously highlighted, delete it.
    Unselect_Current
    .info.top.tippy.unselect configure -state disabled
}

#--------------- Run ---------------#

bind . <Alt-r> {
    .info.top.tippy.run flash
    RunCommand
}

bind . <Alt-s> {
    StepCommand
}

#--------------- Goto Message ---------------#

bind . <Alt-g> {
    End_Search_Mode
    Enter_Goto_Mode
}


bind . <Alt-t> {
    End_Search_Mode
    Enter_Goto_Time_Mode
}


bind . <Return> {
    if ($Goto_Mode==1) {
	set result [catch { set location [expr $minibuf] } ]
	if ($result==0) { Goto_Msg $location }
	End_Goto_Mode
    }

    if ($Goto_Mode==2) {
	Goto_Time_Unparsed $minibuf
	End_Goto_Mode
    }
}

#--------------- Search Listbox ---------------#
# These bindins are intended to provide control and access of the
# listbox in an emacs like manner.

bind . <Control-s> {
    End_Goto_Mode
    Enter_Search_Mode 1
}

bind . <Control-r> {
    Enter_Search_Mode -1
}

bind . <KeyPress> {
    if (([string length %A]>0)&&($Search_Mode==1)||($Goto_Mode>0)) {
	append minibuf %A
	.info.bot.mini.buf configure -text $minibuf
	if ($Search_Mode==1) { Search_For_Message }
    }
}

bind . <Control-g> {
    if ($Search_Mode==1) { End_Search_Mode }
    if ($Goto_Mode>0) { End_Goto_Mode }
}

#------- Traversing the Listbox --------

bind . <Control-v> {
    End_Search_Mode
    .lbo.lbox yview scroll 1 page
    update
}

bind . <Alt-v> {
    End_Search_Mode
    .lbo.lbox yview scroll -1 page
    update
}


#------- Zoom --------

bind . <Control-i> {
    zoomin
}

bind . <Control-o> {
    zoomout
}


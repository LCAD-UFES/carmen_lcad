#######################################################################
############################## Menu ###################################

#---- File Menu -----#

menubutton .mbar.file -padx 5 -text "File" -menu .mbar.file.menu
pack .mbar.file -side left

menu .mbar.file.menu -tearoff 0
.mbar.file.menu add command -label "Open" -command load_file
.mbar.file.menu add command -label "Print Current View" \
	-command save_to_postscript
.mbar.file.menu add command -label "Quit" -command exit


#---- Options Menu -----#

menubutton .mbar.options -padx 5 -text "Options" -menu .mbar.options.menu
pack .mbar.options -side left
menu .mbar.options.menu -tearoff 0

#.mbar.options.menu add checkbutton -label "Show Mouse Time" \
#	-command { if ($mouse_time>0) { set mouse_time 0 } else \
#	{set mouse_time 1 }}

.mbar.options.menu add command -label "Display All Log Data" \
	-command C_Toggle_Display_All

# .mbar.options.menu add command -label "Auto-Grid"

.mbar.options.menu add command -label "REGEXP match" \
	-command Get_Regexp

.mbar.options.menu add command -label "Show Module Host" \
	-command Show_Module_Host

#--- Message Menu ---#

menubutton .mbar.message -padx 15 -text "Message" -menu .mbar.message.menu
menu .mbar.message.menu
pack .mbar.message -side left

#--- Module Menu ----#

menubutton .mbar.module -padx 15 -text Module -menu .mbar.module.menu
menu .mbar.module.menu
pack .mbar.module -side left

#---- Time Menu -----#

menubutton .mbar.time -padx 15 -text Time -menu .mbar.time.menu
menu .mbar.time.menu
pack .mbar.time -side left

.mbar.time.menu add command -label "Display Absolute" \
	-command absolute_command
.mbar.time.menu add command -label "Display Relative" \
	-command relative_command
.mbar.time.menu add checkbutton -label "Toggle Visible Time" \
	-command { Toggle_Visible_Time } -indicatoron 0

proc absolute_command {} {
    global absolute

    set absolute 1
    Update_Visible_Time
}

proc relative_command {} {
    global absolute

    set absolute 0
    Update_Visible_Time
}

#---- Goto Menu -----#

menubutton .mbar.goto -padx 15 -text "Jump" -menu .mbar.goto.menu
menu .mbar.goto.menu
pack .mbar.goto -side left
.mbar.goto.menu add command -label "Time" -command Goto_Time_Menu
.mbar.goto.menu add command -label "Message Number" \
	-command Goto_Msg_Menu


#--- Color Key Menu ---#

set SPACER_FONT "-adobe-helvetica-medium-r-normal--*-10-*-*-*-*-iso*-*"

menubutton .mbar.colors -padx 15 -text "Colors" -menu .mbar.colors.menu
menu .mbar.colors.menu
pack .mbar.colors -side left

.mbar.colors.menu add command \
	-label "Query sent" \
	-background $task_color($QUERY_MESSAGE) \
	-command { Change_Color $QUERY_MESSAGE "Query" }

.mbar.colors.menu add command \
	-label "Query receive" \
	-background $rcv_color($QUERY_MESSAGE) \
	-command { Change_Color $QUERY_MESSAGE "Query" }

.mbar.colors.menu add command -label "" -font $SPACER_FONT \
	-state disabled

.mbar.colors.menu add command \
	-label "Command sent" \
	-background $task_color($COMMAND_MESSAGE) \
	-command { Change_Color $COMMAND_MESSAGE "Command" }

.mbar.colors.menu add command \
	-label "Command receive" \
	-background $rcv_color($COMMAND_MESSAGE) \
	-command { Change_Color $COMMAND_MESSAGE "Command" }

.mbar.colors.menu add command -label "" -font $SPACER_FONT \
	-state disabled

.mbar.colors.menu add command \
	-label "Inform sent" \
	-background $task_color($INFORM_MESSAGE) \
	-command { Change_Color $INFORM_MESSAGE "Inform" }

.mbar.colors.menu add command \
	-label "Inform receive" \
	-background $rcv_color($INFORM_MESSAGE) \
	-command { Change_Color $INFORM_MESSAGE "Inform" }

.mbar.colors.menu add command -label "" -font $SPACER_FONT \
	-state disabled

.mbar.colors.menu add command \
	-label "Broadcast sent" \
	-background $task_color($BROADCAST_MESSAGE) \
	-command { Change_Color $BROADCAST_MESSAGE "Broadcast" }

.mbar.colors.menu add command \
	-label "Broadcast receive" \
	-background $rcv_color($BROADCAST_MESSAGE) \
	-command { Change_Color $BROADCAST_MESSAGE "Broadcast" }

.mbar.colors.menu add command -label "" -font $SPACER_FONT \
	-state disabled

.mbar.colors.menu add command \
	-label "Goal sent" \
	-background $task_color($GOAL_MESSAGE) \
	-command { Change_Color $GOAL_MESSAGE "Goal" }

.mbar.colors.menu add command \
	-label "Goal receive" \
	-background $rcv_color($GOAL_MESSAGE) \
	-command { Change_Color $GOAL_MESSAGE "Goal" }

.mbar.colors.menu add command -label "" -font $SPACER_FONT \
	-state disabled

.mbar.colors.menu add command \
	-label "Pending bar" \
	-background $PEND_BAR_COLOR


#--- Unused Menu ---#

if (0) {

menubutton .mbar.options -padx 10 -text Options -menu .mbar.options.menu

menu .mbar.options.menu
.mbar.options.menu add command -label "Select Messages"
.mbar.options.menu add command -label "Colors"
.mbar.options.menu add command -label "Activity Border"

menubutton .mbar.help -text Help -menu .mbar.help.menu
pack .mbar.help .mbar.help -side right


}

#menu .mbar.help.menu
#.mbar.help.menu add command -label "Sorry, no help yet"

######################################################################

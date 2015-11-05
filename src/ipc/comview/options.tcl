# -------------------------------------------------------------------- #
#
# Comview -- TCA Communication Graphic Tool
# Colors and Settings
#
# -------------------------------------------------------------------- #

set EVENT_LINE_COLOR "black"

# the list of all used colors is maintained inorder to restore the
# correct outline color on commnication widgets after saving a
# postscript image of the canvas.

set COLOR_LIST [list \
	task_color($REPLY_MESSAGE) \
	task_color($QUERY_MESSAGE) \
	task_color($INFORM_MESSAGE) \
	task_color($COMMAND_MESSAGE) \
	task_color($BROADCAST_MESSAGE) \
	task_color($GOAL_MESSAGE) \
	rcv_color($QUERY_MESSAGE) \
	rcv_color($INFORM_MESSAGE) \
	rcv_color($COMMAND_MESSAGE) \
	rcv_color($BROADCAST_MESSAGE) \
	rcv_color($GOAL_MESSAGE)]
	

# OUTLINE determines whether the postscript output has a black outline
# around all graphical activity.
set OUTLINE 1

# These are the colors of the rectangles for outlining the source and
# destination modules in a communication activity.
set MODULE_CALL_OUTLINE_COLOR "red"
set MODULE_CALLED_OUTLINE_COLOR "blue"

# These units are pixels:
set MODULE_HEIGHT 40
set MODULE_SPACE 22
#set MODULE_HEIGHT 5
#set MODULE_SPACE 2
set PEND_ARROW_OFFSET 6

set PENDING_MODULE_HEIGHT 6
set MODULE_LABEL_WIDTH 250

# Some settings dealing with the canvas layout geometry (pixels):

# The initial height and width of the canvas (will be dynamically
# expanded). Making larger will allow more slack in scrolling.
# It's commented out to enable the user to modify the show_width, and
# show_height in their settings file (those values are then used
# within comview.tcl to compute CANVAS_WIDTH and CANVAS_HEIGHT).
#

# Geometry settings for 800x600 laptop display
if ($GEOMETRY_SIZE==1) {
    set CANVAS_SHOW_WIDTH 600
    set CANVAS_SHOW_HEIGHT 220

    # Follow offset is the distance across the canvas that incoming data
    # should appear at when scroll-lock is active.
    set FOLLOW_OFFSET 600.0

    # Number of elements viewable in the log window (listbox at the bottom
    # of the display).
    set LISTBOX_HEIGHT 4
    set LISTBOX_WIDTH 84

    set MODULE_CANVAS_WIDTH 264
}

if ($GEOMETRY_SIZE==2) {
    # The show width and show height are the size of the canvas which is
    # actually visible at any given time (ie. the canvas window size).
    set CANVAS_SHOW_WIDTH 800
    set CANVAS_SHOW_HEIGHT 390

    # Follow offset is the distance across the canvas that incoming data
    # should appear at when scroll-lock is active.
    set FOLLOW_OFFSET 790.0

    # Number of elements viewable in the log window (listbox at the bottom
    # of the display).
    set LISTBOX_HEIGHT 7
    set LISTBOX_WIDTH 98

    set MODULE_CANVAS_WIDTH 264
}

# SHOW_IGNORED_MESSAGES (0 or 1) determines whether or not an ignored
# message (ie. not displayed on the graphics window) should still be
# displayed in the log window.
set LOG_IGNORED_MESSAGES 0

# The default scale settings:
set Default_Scale_Factor 10.0

# this setting is not currently used for anything:
set Time_Scale_Factor 10

# When a zoom in or out is done, how much should the canvas scale:
set Zoom_Amount 1.4

# The height (number of rows) in the listbox for a particular message
# type selected from  the "Message" menu.
set MESSAGE_LISTBOX_HEIGHT 12

# Display_Time is the initial state for whether the time for the
# leftmost and rightmost visible points on the canvas are displayed
# above those positions (set to 0 on slow systems for speed reasons).
set DISPLAY_TIME 1

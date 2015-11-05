########################################################################
################# Some Global Constants and Variables ##################

set tcl_precision 17

set CANVAS_WIDTH [expr $CANVAS_SHOW_WIDTH*1.0]
set CANVAS_HEIGHT [expr $CANVAS_SHOW_HEIGHT*1.0]

# MODULE_LABEL_WIDTH is defined in options.tcl
set MODULE_X_OFFSET [expr ($MODULE_LABEL_WIDTH/2)+5]

set HALF_LISTBOX_HEIGHT 3

set MODULE_DISTANCE [expr $MODULE_HEIGHT+$MODULE_SPACE]

set highlight_multiple 0
set absolute 1
set RunOK 0
set CurrentShowAll "0"
set cur 0

# current_highlight will keep track of the current highlight selection
# so that after a scale or module_move event occurs (which requires
# that the current_highlight be deleted) we can redraw a new one.
set current_highlight -1

if ($Scale_Factor==-1) { 
    set Scale_Factor [expr 1.0/$Default_Scale_Factor]
} else {
    set Scale_Factor [expr 1.0/$Scale_Factor]
}

set Search_Mode 0
set search_start_loc 0
set last_lbox_find 0

set Goto_Mode 0

set new_selection 0
set SHOW_HOSTNAME 1

set from_module 0
set to_module 0

set keep_update 1

# this variable determines whether the scroll bar follows the front of
# the incoming communication data, resulting in a scrolling display.
set follow_production 1

# call_count counts the number of module calls. A module "call" is an
# invocation of a module, and consists of all the activity of that
# module until it completes.
set call_count 1

set msg_count 1
set time_count 1

set msg_number -1

# the number of modules dynamically changes
set NUMBER_OF_MODULES 0

set time_position 0

set Goto_Msg_Menu_Exists 0
set Goto_Time_Menu_Exists 0

set last_search_string ""
set minibuf ""

# ----------------- Determine the settings filename ------------------

set result [expr [string length $settings_file]>0]

if ($FILE_SELECTED) {
    # remove any path information from the filename (for use in displaying
    # the filename above the graphical display).
    set fname [string range $filename \
	    [expr [string last "/" $filename]+1] \
	    [string length $filename]]

    # create the log file name to search for:
    
    # loc is the location of the "." in the filename
    set loc [expr [string last "." $filename]-1]
    if ($loc<0) { set loc [string length $filename] }
    # raw_filename is the filename excluding the ".log", if present.
    # ie. for "foo.log" raw_filename is "foo"
    set raw_filename [string range $filename 0 $loc]

    if ($result) {
    } else {
	set settings_file [string range $filename 0 $loc]
	append settings_file ".set"
    }
} else {
    set fname "No File Opened"
}

# --------------------------------------------------------------------

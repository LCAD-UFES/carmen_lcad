# cregexp.tcl
# 
# Code to Check for a Regular Expression in the log file
#
# Should create a window with a number of options:
#   a) Stop when regexp matches
#   b) create event line when regexp matches

set GLOBAL_REGEXP ""

proc Check_Regexp { str } {
    global GLOBAL_REGEXP RunOK

    set result [regexp $GLOBAL_REGEXP $str]
    if ($result==1) { set RunOK 0 }
}

proc Get_Regexp {} {
    global GLOBAL_REGEXP

    toplevel .re
    wm title .re "Match Regular Expression"

    text .re.str -width 50 -height 1
    .re.str insert 1.0 $GLOBAL_REGEXP
    pack .re.str

    button .re.ok -text "Ok" \
	    -command { re_command }
    button .re.ignore -text "Ignore REGEXP" -command { re_ignore }
    button .re.cancel -text "Cancel" -command { destroy .re }

    bind .re <Return> { re_command }

    pack .re.ok -side left
    pack .re.ignore -side right
    pack .re.cancel -side right
}

#-----------------------------------------------------------------------
# re_command is called when the "Ok" button is hit from the "Match
# Regular Expression" menu (or when Return is hit).
#-----------------------------------------------------------------------

proc re_command {} {
    global GLOBAL_REGEXP

    C_Match_Regexp 1
    set GLOBAL_REGEXP [.re.str get 1.0 1.end]
    destroy .re
}

proc re_ignore {} {
    global GLOBAL_REGEXP

    set GLOBAL_REGEXP ""
    C_Match_Regexp 0
    destroy .re
}

# -------------------------------------------------------------------- #
#
# Comview -- TCA Communication Graphic Tool
# Greg Whelan, whelan@cs.cmu.edu
# Carnegie Mellon University
#
# main.tcl
#
# -------------------------------------------------------------------- #

# the order that these files are loaded is important, there are many
# definitions in earlier files that are required even to just source
# later files.

source $COMVIEW_DIRECTORY/queues.tcl
source $COMVIEW_DIRECTORY/tca-header.tcl
source $COMVIEW_DIRECTORY/colors.tcl
source $COMVIEW_DIRECTORY/options.tcl
source $COMVIEW_DIRECTORY/keyboard.tcl
source $COMVIEW_DIRECTORY/FSBox.tcl
source $COMVIEW_DIRECTORY/set-colors.tcl
source $COMVIEW_DIRECTORY/cregexp.tcl
source $COMVIEW_DIRECTORY/vars.tcl
source $COMVIEW_DIRECTORY/layout.tcl
source $COMVIEW_DIRECTORY/comview.tcl
source $COMVIEW_DIRECTORY/logwindow.tcl
source $COMVIEW_DIRECTORY/config.tcl
source $COMVIEW_DIRECTORY/menu.tcl
source $COMVIEW_DIRECTORY/mouse.tcl

catch { source $settings_file }

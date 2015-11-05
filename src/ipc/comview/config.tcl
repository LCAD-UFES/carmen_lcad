# -------------------------------------------------------------------- #
#
# TCA Communications Graphics Tool
# Ignoring messages names and messages from a module.
#
# -------------------------------------------------------------------- #

proc module {number name} {
    global module_order
    
    set module_order($number) $name
}

proc ignore_module {module} {
    global ignored_modules

    set ignored_modules($module) 1
}

proc clear_ignore_module {module} {
    global ignored_modules

    set ignored_modules($module) 0
}

proc ignore_message {message} {
    global ignored_messages

    set ignored_messages($message) 1
}

proc clear_ignore_message {message} {
    global ignored_messages

    set ignored_messages($message) 0
}

proc ignored_module_p {module} {
    global ignored_modules

    # catch returns 1 when an error occurred, 0 = no error.
    set c [catch { set result $ignored_modules($module) }]
    if ($c==1) { set result 0 }

    # result should now be 0=message ignored, 1=not ignored
    return $result
}

proc ignored_message_p {message} {
    global ignored_messages

    # catch returns 1 when an error occurred, 0 = no error.
    set c [catch { set result $ignored_messages($message) }]
    if ($c==1) { set result 0 }

    # result should now be 0=message ignored, 1=not ignored
    return $result
}

set sf_result [catch { source $settings_file }]

if ($sf_result>0) {
    # puts "Nonexistent settings file: $settings_file"
}

# At this point what's were going to do is ignore any messages which
# are sent by the other modules which are not to be used in the
# display.

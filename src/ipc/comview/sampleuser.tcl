
#--- Sample User Code --

set my_count 0

proc User_Event_Hook { from_module to_module message_type name } {
    global my_count
    
    if (([string compare $from_module "fscdemo"]==0)&&([string compare $to_module "console"]==0)) {
	incr my_count
	if ($my_count==2) { 
#	    Create_Event_Line "second uno->dos"
	}
    }
}

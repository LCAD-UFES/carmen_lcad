#########################################################################
# Queues for TCL (this is very ugly, should be implemented in C)
# perhaps implement with Lists in TCL
#########################################################################

proc make_queue { queue } {
    global global_queues

    set global_queues($queue,0) 2
    set global_queues($queue,1) 2
}

proc enqueue { queue element } {
    global global_queues

    set in $global_queues($queue,0)
    incr global_queues($queue,0)
    set global_queues($queue,$in) $element
}

proc dequeue { queue } {
    global global_queues

    set out $global_queues($queue,1)
    set in $global_queues($queue,0)
    
    if ($out==$in) { 
	error "Error: Queue empty"
    }

    incr global_queues($queue,1)
    return $global_queues($queue,$out)
}


#########################################################################
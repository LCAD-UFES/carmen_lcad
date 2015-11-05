# -------------------------------------------------------------------- #
#
# TCA Communications Graphics Tool
# TCA Constants
#
# -------------------------------------------------------------------- #

# These values should correspond with the definitions in MsgData.h
# from the msgType enumerated type.

set QUERY_MESSAGE       1
set GOAL_MESSAGE        2
set COMMAND_MESSAGE     3
set INFORM_MESSAGE      4
set EXCEPTION_MESSAGE   5
set KILLED_MESSAGE      6
set INTERVAL_MESSAGE    7
set POINT_MESSAGE       8
set FIRE_MESSAGE        9
set REPLY_MESSAGE       10
set FAILURE_MESSAGE     11
set SUCCESS_MESSAGE     12
set BYPASS_MESSAGE      13
set PCONSTRAINT_MESSAGE 14
set RETRY_MESSAGE       15
set WILL_KILL_MESSAGE   16
set TCONSTRAINT_MESSAGE 17
set WARNING_MESSAGE     18
set CONNECT_MESSAGE     19
set DISCONNECT_MESSAGE  20
set BROADCAST_MESSAGE   21

# These values should correspond with the definitions in MsgData.h
# from the msgStatus enumerated type. Currently the only status
# message that's used by comview is the pending status message.

set PENDING_STATUS 3

# Module State (Comview specific)

set INACTIVE_STATE 0
set ACTIVE_STATE 1
set WAITING_STATE 101
set PENDING_STATE 3

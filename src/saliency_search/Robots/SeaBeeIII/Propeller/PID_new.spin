CON
FWD_RIGHT_THRUSTER = 3
FWD_LEFT_THRUSTER = 1
DEPTH_RIGHT_THRUSTER = 4
DEPTH_LEFT_THRUSTER = 2
STRAFE_FRONT_THRUSTER = 0
STRAFE_BACK_THRUSTER = 5

VAR
long stack[50]
long cognum

long cur_pos_heading        'Real Position
long set_pos_heading        'Set Point
long KK_heading              'PID Gain
long output_heading         'PID Output

long cur_pos_depth        'Real Position
long set_pos_depth        'Set Point
long KK_depth              'PID Gain
long output_depth         'PID Output

long KP_heading
long KD_heading
long KI_heading

long KP_depth
long KD_depth
long KI_depth

long I_heading
long I_heading_max
long I_heading_min
long prev_Error_heading

long I_depth
long I_depth_max
long I_depth_min
long prev_Error_depth

long prev_heading
long prev_depth

long Error_heading
long Error_depth

long pid_enablepH
long pid_enablepD
long dutyp[9]        'Duty cycle for motors 0-5, Marker droppers 6-7, and aux motor 8
long dirp[9]         'Direction for motors 0-5, Marker droppers 6-7, and aux motor 8
long depthGranp
long headingGranp

long desired_speed_addr


PUB Start(address1)
  setAddress(address1)
  initPID
  cognum := cognew(updatePID, @stack)

PUB Stop
  cogstop(cognum)

PUB setAddress(address1)
  desired_speed_addr :=         address1+4*21
  dutyp[0] :=                   address1+4*62
  dutyp[1] :=                   address1+4*63
  dutyp[2] :=                   address1+4*64
  dutyp[3] :=                   address1+4*65
  dutyp[4] :=                   address1+4*66
  dutyp[5] :=                   address1+4*67
  dutyp[6] :=                   address1+4*68
  dutyp[7] :=                   address1+4*69
  dutyp[8] :=                   address1+4*70
  dirp[0] :=                    address1+4*71  
  dirp[1] :=                    address1+4*72
  dirp[2] :=                    address1+4*73
  dirp[3] :=                    address1+4*74
  dirp[4] :=                    address1+4*75
  dirp[5] :=                    address1+4*76
  dirp[6] :=                    address1+4*77
  dirp[7] :=                    address1+4*78
  dirp[8] :=                    address1+4*79

  pid_enablepH :=               address1+4*82
  set_pos_heading :=            address1+4*19
  cur_pos_heading :=            address1+4*30
  output_heading :=             address1+4*40
  headingGranp :=               address1+4*85

  pid_enablepD :=               address1+4*83
  set_pos_depth :=              address1+4*20
  cur_pos_depth :=              address1+4*9   
  output_depth :=               address1+4*39
  depthGranp :=                 address1+4*84

  KK_heading :=                 address1+4*38 
  KP_heading :=                 address1+4*42
  KD_heading :=                 address1+4*43
  KI_heading :=                 address1+4*44

  KK_depth :=                   address1+4*37
  KP_depth :=                   address1+4*45
  KD_depth :=                   address1+4*46
  KI_depth :=                   address1+4*47   


Pub initPID
I_heading := 0
I_heading_max := 2000
I_heading_min := -2000
prev_Error_heading := -1
prev_heading := long[set_pos_heading]
 
I_depth := 0
I_depth_max := 2000
I_depth_min := -2000
prev_Error_depth := -1
prev_depth := long[set_pos_depth]

Pub updatePID | D_heading, D_depth, temp,currentH,setH
repeat
  if long[pid_enablepH] == 1
    updateMotorsSpeed
  '''''''''''''''''''''''''''''''HEADING PID'''''''''''''''''''''''''''''''
  'Check for updated heading
  if long[set_pos_heading] <> prev_heading
    prev_heading := long[cur_pos_heading]
    I_heading := 0
    prev_Error_heading := -1  
  'Calculate the error
  'Error_heading := long[set_pos_heading]-long[cur_pos_heading]
  'Normalize along 0 degree
  if long[cur_pos_heading]<180
    currentH := 0
    setH := long[set_pos_heading]-long[cur_pos_heading]
  else
    currentH := 0
    setH := (long[set_pos_heading]+(360-long[cur_pos_heading]))//360
  'Calculate error
  if setH<180
    Error_heading := setH
  else
    Error_heading := -1*(360-setH)

  'Calculate the integral of the error, and bound it between I_heading_min and I_heading_max to prevent spool-up
  '(Make sure to set I_heading to 0 at startup, and whenever a new set_pos_heading is requested)
  I_heading := I_heading + Error_heading
  I_heading #>= I_heading_min
  I_heading <#= I_heading_max   
  
  'If there is no previous heading reading, initialize it to the current heading so that
  '  the derivative will be 0.
  '((Make sure that the prev_error_heading is initialized to -1 in the main function at startup))
  '(((Also, be sure to reset the prev_P_heading to -1 whenever a new set_pos_heading is requested)))
  if (prev_Error_heading == -1)
    prev_Error_heading := Error_heading

  'Calculate the derivative of the error
  D_heading := Error_heading - prev_Error_heading
  
  'Store the history of the heading
  prev_Error_heading := Error_heading

  'Now calculate the control signal 'u'
  temp := (long[KP_heading] * Error_heading) + (long[KI_heading] * I_heading) - (long[KD_heading] * D_heading)
  long[output_heading] := temp/long[headingGranp]
  if long[pid_enablepH] == 1
    updateMotorsHeading   
  
  '''''''''''''''''''''''''''''''DEPTH PID'''''''''''''''''''''''''''''''
  'Check for updated depth
  if long[set_pos_depth] <> prev_depth
    prev_depth := long[cur_pos_depth]
    I_depth := 0
    prev_Error_depth := -1
  'Calculate the error
  Error_depth := long[set_pos_depth] - long[cur_pos_depth]

  'Calculate the integral of the error, and bound it between I_depth_min and I_depth_max to prevent spool-up
  '(Make sure to set I_depth to 0 at startup, and whenever a new set_pos_depth is requested)
  I_depth := I_depth + Error_depth
  I_depth #>= I_depth_min
  I_depth <#= I_depth_max   
  
  'If there is no previous depth reading, initialize it to the current depth so that
  '  the derivative will be 0.
  '((Make sure that the prev_error_depth is initialized to -1 in the main function at startup))
  '(((Also, be sure to reset the prev_P_depth to -1 whenever a new set_pos_depth is requested)))
  if (prev_Error_depth == -1)
    prev_Error_depth := Error_depth

  'Calculate the derivative of the error
  D_depth := Error_depth - prev_Error_depth
  
  'Store the history of the depth
  prev_Error_depth := Error_depth

  'Now calculate the control signal 'u'
  temp := (long[KP_depth] * Error_depth) + (long[KI_depth] * I_depth) - (long[KD_depth] * D_depth)
  long[output_depth] := temp/long[depthGranp]
  if long[pid_enablepD] == 1
    updateMotorsDepth

{{
mod by 360
-(val+360-desired)
(val-desired)

}}
PUB updateMotorsDepth | speed
  speed := long[output_depth]
  speed #>= -100
  speed <#= 100
  setMotor(DEPTH_LEFT_THRUSTER,speed)
  setMotor(DEPTH_RIGHT_THRUSTER,speed)
  

PUB updateMotorsHeading | speed
  speed := long[output_heading]
  speed #>= -100
  speed <#= 100                                
  setMotor(STRAFE_BACK_THRUSTER,speed)
  speed := -1*speed
  setMotor(STRAFE_FRONT_THRUSTER,speed)

PUB updateMotorsSpeed
  setMotor(FWD_RIGHT_THRUSTER,long[desired_speed_addr])
  setMotor(FWD_LEFT_THRUSTER,-1*long[desired_speed_addr])

PUB setMotor(motor,speed)
'Set motor speed and direction
'Speed 0-100
  long[dutyp[motor]] := ||(speed)
  if speed < 0
    long[dirp[motor]] := 1
  else
    long[dirp[motor]] := 0    

OBJ
  Serial : "FullDuplexSerial" 'Serial Interface to Computer
VAR
  long CompCommStack[40]

  long accel_data_addr[3]
  {{
    accel_data[0]=x status
    accel_data[1]=y status
    accel_data[2]=z status
  }}

  long adc_data_addr[15]
{{
  adc_data[0]=ADC1_ADDR AND READ_AD1           adc_data[8]=ADC2_ADDR AND READ_AD1
  adc_data[1]=ADC1_ADDR AND READ_AD2           adc_data[9]=ADC2_ADDR AND READ_AD2
  adc_data[2]=ADC1_ADDR AND READ_AD3           adc_data[10]=ADC2_ADDR AND READ_AD3
  adc_data[3]=ADC1_ADDR AND READ_AD4           adc_data[11]=ADC2_ADDR AND READ_AD4
  adc_data[4]=ADC1_ADDR AND READ_AD5           adc_data[12]=ADC2_ADDR AND READ_AD5
  adc_data[5]=ADC1_ADDR AND READ_AD6           adc_data[13]=ADC2_ADDR AND READ_AD6
  adc_data[6]=ADC1_ADDR AND READ_AD7           adc_data[14]=ADC2_ADDR AND READ_AD7
  adc_data[7]=ADC1_ADDR AND READ_AD8           adc_data[15]=ADC2_ADDR AND READ_AD8
}}

  ''Command Data
  long desired_heading_addr
  long desired_depth_addr
  long desired_speed_addr
  long marker_drop_addr[2]

   'Compass Data
  long comp_accel_addr[3]
  {
    comp_accel_data[0] = x status
    comp_accel_data[1] = y status
    comp_accel_data[2] = z status
  }
  long comp_mag_addr[3]
  {
   Mx, My, Mz
  }
  long comp_heading_addr[3]
  {
   Head, Pitch, Roll
  }
  long comp_tilt_addr[3]
  {
   Pitch, Roll, Temp
  }
  long opmode_addr
  'PID Variables
  long KK_depth_addr
  long KK_heading_addr
  long output_depth_addr
  long output_heading_addr
  long integral_time_addr
  'PID Constants
  long KP_heading_addr
  long KD_heading_addr
  long KI_heading_addr
  long KP_depth_addr
  long KD_depth_addr
  long KI_depth_addr
  long compcog
  'Battery
  long battery1_addr
  long battery2_addr
  long battery3_addr
  long battery4_addr
  'Compass
  long compass_cal_addr
  

  long enablep[9]      'Enable for motors 0-5, Marker droppers 6-7, and aux motor 8
  long dutyp[9]        'Duty cycle for motors 0-5, Marker droppers 6-7, and aux motor 8
  long dirp[9]         'Direction for motors 0-5, Marker droppers 6-7, and aux motor 8 
  long frequencyp
  long delayp
  long pid_enableHp
  long pid_enableDp
  long depthGranp
  long headingGranp
  long killstatusp

PUB Start(address)
  CompCog := cognew(CompComm(address), @CompCommStack)

PUB CompComm(address) | command, send, temp
'Get Global Variable Address
  setaddress(address)

  Serial.start(31,30,0,57600) 
  'motorCtl.start


'Get Computer Command
  repeat    
    command := Serial.rx
    'if(command == -1)
      'disablePID
    'Send data to Computer
{{00-Send Data
}}    
    if command == $00
      sendFull
{{0F-Set
heading_lower,heading_upper,depth,speed,marker_drop1, marker_drop2
}}
    elseif command == $0F
      setStuff
      
{{FF-Set Motor
}}
    elseif command == $FF
      setmotor
'PID Stuff
{{20-KK_depth
}}
    elseif command == $20
      temp := Serial.rx
      long[KK_depth_addr] := ~temp
{{21-KP_depth
}}
    elseif command == $21
      temp := Serial.rx
      long[KP_depth_addr] := ~temp
{{22-KI_depth
}}
    elseif command == $22
      temp := Serial.rx
      long[KI_depth_addr] := ~temp
{{23-KD_depth
}}
    elseif command == $23
      temp := Serial.rx
      long[KD_depth_addr] := ~temp
{{24-Output Depth PID
}}
    elseif command == $24
      Serial.tx(long[output_depth_addr])
{{25-Depth Granularity
}}      
    elseif command ==  $25
      temp := Serial.rx
      'if(temp == -1)
        'disablePID
      long[depthGranp]:=temp     
    
{{30-KK_heading
}}
    elseif command == $30
      temp := Serial.rx
      'if(temp == -1)
        'disablePID
      long[KK_heading_addr] := ~temp
{{31-KP_heading
}}
    elseif command == $31
      temp := Serial.rx
      'if(temp == -1)
        'disablePID
      long[KP_heading_addr] := ~temp
{{32-KI_heading
}}
    elseif command == $32
      temp := Serial.rx
      'if(temp == -1)
        'disablePID
      long[KI_heading_addr] := ~temp
{{33-KD_heading
}}
    elseif command == $33
      temp := Serial.rx
      'if(temp == -1)
        'disablePID
      long[KD_heading_addr] := ~temp
{{34-output heading pid
}}
    elseif command == $34
      Serial.tx(long[output_heading_addr])
{{35-Heading granularity
}}      
    elseif command == $35
      temp := Serial.rx
      'if(temp == -1)
        'disablePID
      long[headingGranp]:=temp    
{{E0-Compass Cal Start
}}
    if command == $E0
      long[compass_cal_addr] := $01
{{E1-Compass Cal End
}}
    elseif command == $E1
      long[compass_cal_addr] := $02
{{60-Set PID Heading Status 0-Off 1-On
}}      
    elseif command == $60
      long[pid_enableHp] := serial.rx
      'if(temp == -1)
        'disablePID
{{61-Set PID Depth Status, 0-Off 1-On
}}
    elseif command == $61      
      long[pid_enableDp] := serial.rx
      'if(temp == -1)
        'disablePID
    elseif command == $05
      compass
    elseif command == $06
      compassReading
    elseif command == $0B
      temp := 0
      temp := Serial.rx
      'if(temp == -1)
        'disablePID
      temp <<= 8
      temp += Serial.rx
      'if(temp == -1)
        'disablePID
      long[desired_heading_addr] := temp
    elseif command == $0C
      temp := 0
      temp := Serial.rx
      ''if(temp == -1)
        'disablePID
      temp <<= 8
      temp += Serial.rx
      ''if(temp == -1)
        'disablePID
      long[desired_depth_addr] := temp
    elseif command == $0D
      temp := 0
      temp := Serial.rx
      long[desired_speed_addr] := ~temp
    elseif command == $0E
      setheading

PUB setheading | temp
  temp := 0
  temp := Serial.rx
  temp <<= 8
  temp += Serial.rx
  long[comp_heading_addr][0] := temp*10

PUB setstuff | temp
{{
  heading_upper,heading_lowre,depth_upper,depth_lower,speed,marker_drop1, marker_drop2
}}
      temp := 0
      temp := Serial.rx
      ''if(temp == -1)
        'disablePID
      temp <<= 8
      temp += Serial.rx
      ''if(temp == -1)
        'disablePID
      long[desired_heading_addr] := temp*10
      temp := 0
      temp := Serial.rx
      ''if(temp == -1)
        'disablePID
      temp <<= 8
      temp += Serial.rx
      ''if(temp == -1)
        'disablePID
      long[desired_depth_addr] := temp
      temp := 0
      temp := Serial.rx
      ''if(temp == -1)
        'disablePID
      long[desired_speed_addr] := ~temp
      long[marker_drop_addr][0] := Serial.rx
      ''if(temp == -1)
        'disablePID
      long[marker_drop_addr][1] := Serial.rx
      ''if(temp == -1)
        'disablePID

PUB setmotor| motor, speed, x

  motor := Serial.rx
  'if(motor == -1)
        'disablePID
  x := Serial.rx
  'if(x == -1)
        'disablePID
  speed := ~x
  'dir := Serial.rx
  setMotorValue(motor, speed) 

PUB setMotorValue(motor, speed)
{{Set motor speed and direction
Speed 0-100
Direction 1-Forward, 0-Backward}}
  long[dutyp[motor]] := ||(speed)
  if speed < 0
    long[dirp[motor]] := 1
  else
    long[dirp[motor]] := 0
 
  
Pub setaddress(address) | i
  accel_data_addr[0] :=         address+4*0
  accel_data_addr[1] :=         address+4*1
  accel_data_addr[2] :=         address+4*2
  repeat i from 0 to 15
    adc_data_addr[i] :=         address+4*3+4*i
  desired_heading_addr :=       address+4*19
  desired_depth_addr :=         address+4*20
  desired_speed_addr :=         address+4*21
  marker_drop_addr[0] :=        address+4*22
  marker_drop_addr[1] :=        address+4*23
  repeat i from 0 to 2
    comp_accel_addr[i] :=       address+96+4*i
    comp_mag_addr[i] :=         address+108+4*i
    comp_heading_addr[i] :=     address+120+4*i
    comp_tilt_addr[i] :=        address+132+4*i
  opmode_addr :=                address+4*36
  KK_depth_addr :=              address+4*37
  KK_heading_addr :=            address+4*38
  output_depth_addr :=          address+4*39
  output_heading_addr :=        address+4*40
  integral_time_addr :=         address+4*41
  KP_heading_addr :=            address+4*42
  KD_heading_addr :=            address+4*43
  KI_heading_addr :=            address+4*44
  KP_depth_addr :=              address+4*45
  KD_depth_addr :=              address+4*46
  KI_depth_addr :=              address+4*47
  battery1_addr :=              address+4*48
  battery2_addr :=              address+4*49
  battery3_addr :=              address+4*50
  battery4_addr :=              address+4*51
  compass_cal_addr :=           address+4*52
  enablep[0] :=                 address+4*53
  enablep[1] :=                 address+4*54
  enablep[2] :=                 address+4*55
  enablep[3] :=                 address+4*56
  enablep[4] :=                 address+4*57
  enablep[5] :=                 address+4*58
  enablep[6] :=                 address+4*59
  enablep[7] :=                 address+4*60
  enablep[8] :=                 address+4*61
  dutyp[0] :=                   address+4*62
  dutyp[1] :=                   address+4*63
  dutyp[2] :=                   address+4*64
  dutyp[3] :=                   address+4*65
  dutyp[4] :=                   address+4*66
  dutyp[5] :=                   address+4*67
  dutyp[6] :=                   address+4*68
  dutyp[7] :=                   address+4*69
  dutyp[8] :=                   address+4*70
  dirp[0] :=                    address+4*71  
  dirp[1] :=                    address+4*72
  dirp[2] :=                    address+4*73
  dirp[3] :=                    address+4*74
  dirp[4] :=                    address+4*75
  dirp[5] :=                    address+4*76
  dirp[6] :=                    address+4*77
  dirp[7] :=                    address+4*78
  dirp[8] :=                    address+4*79
  frequencyp :=                 address+4*80
  delayp :=                     address+4*81
  pid_enableHp :=               address+4*82
  pid_enableDp :=               address+4*83
  depthGranp :=                 address+4*84
  headingGranp :=               address+4*85
  killstatusp :=                address+4*86
PUB sendFull | temp
      Serial.tx(long[accel_data_addr][0])
      Serial.tx(long[accel_data_addr][1])
      Serial.tx(long[accel_data_addr][2])

      temp := 0
      temp := long[adc_data_addr][0]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][1]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][2]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
     temp := 0
      temp := long[adc_data_addr][3]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][4]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][5]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][6]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][7]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][8]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][9]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][10]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][11]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][12]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][13]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][14]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[adc_data_addr][15]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)

     ''Command Data
      temp := 0
      temp := long[desired_heading_addr]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      temp := 0
      temp := long[desired_depth_addr]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)
      Serial.tx( long[desired_speed_addr]   )
      Serial.tx( long[marker_drop_addr][0]  )
      Serial.tx( long[marker_drop_addr][1]  )

       'Compass Data
      temp := long[comp_accel_addr][0]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte
      temp := long[comp_accel_addr][1]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte
      temp := long[comp_accel_addr][2]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte

      temp := long[comp_mag_addr][0]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte
      temp := long[comp_mag_addr][1]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte
      temp := long[comp_mag_addr][2]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte

      temp := long[comp_heading_addr][0]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte
      temp := long[comp_heading_addr][1]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte
      temp := long[comp_heading_addr][2]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte

      temp := long[comp_tilt_addr][0]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte
      temp := long[comp_tilt_addr][1]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte
      temp := long[comp_tilt_addr][2]
      Serial.tx(temp) 'lower byte
      temp >>= 8
      Serial.tx(temp) 'upper byte

      'battery status
      Serial.tx(long[battery1_addr])
      Serial.tx(long[battery2_addr])
      Serial.tx(long[battery3_addr])
      Serial.tx(long[battery4_addr])

      'PID Variables
      Serial.tx(long[KK_heading_addr])
      Serial.tx(long[KP_heading_addr])
      Serial.tx(long[KD_heading_addr])
      Serial.tx(long[KI_heading_addr])
      temp := 0
      temp := long[output_heading_addr]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)  
      Serial.tx(long[KK_depth_addr])
      Serial.tx(long[KP_depth_addr])
      Serial.tx(long[KD_depth_addr])
      Serial.tx(long[KI_depth_addr])
      temp := 0
      temp := long[output_depth_addr]
      Serial.tx(temp)
      temp >>=8
      Serial.tx(temp)

      'Send killstatus
      Serial.tx(long[killstatusp])


PUB Stop
''Stops the Cog and the PID controller
cogstop(compcog)

PUB compass
  repeat
    Serial.str(String("Heading: "))
    Serial.dec(long[output_heading_addr])
    Serial.str(String(" Depth: "))
    Serial.dec(long[output_depth_addr])
    Serial.tx(10)
    Serial.tx(13)
    
    waitcnt(4_000_000+cnt)

PUB compassReading
  repeat
    Serial.str(String("Heading: "))
    Serial.dec(long[comp_heading_addr[0]])
    Serial.tx(10)
    Serial.tx(13)
    
    waitcnt(4_000_000+cnt)

PUB disablePID | motor
     long[pid_enableHp] := 0
     long[pid_enableDp] := 0
     repeat motor from 0 to 8
       long[dutyp[motor]] := 0

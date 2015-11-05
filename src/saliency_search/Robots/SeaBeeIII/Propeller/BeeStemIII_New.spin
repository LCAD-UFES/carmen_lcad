{{ Motor Direction and PWM ports
 Clk is at 10MHz, need 100Hz, so restart every 100_000 clk cycles
 Note counter is 32 bits and a long is also 32 bits, no conversion needed

 Motor 0: PWM = 3;  Dir = 4
 Motor 1: PWM = 16;  Dir = 2
 Motor 2: PWM = 14;  Dir = 15

 Motor 3: PWM = 42;  Dir = 41
 Motor 4: PWM = 11;  Dir = 43
 Motor 5: PWM = 9;  Dir = 10

 Motor 6: PWM = 12;  Dir = 13
 Motor 7: PWM = 19;  Dir = 20
 Motor 8: PWM = 44; Dir = 1

 KILL SWITCH = P22 -- Set this high to kill all of the motors
}}

CON
  _CLKMODE = XTAL1 + PLL8x
  _XINFREQ = 10_000_000
  I2C_SDA = 29
  I2C_SCL = 28
  'A-D Const
  WRITE = 0
  READ = 1
  'A-D Converter
  ADC1_ADDR = %010_00110        'AS Pin to AGND
  ADC2_ADDR = %010_01000        'AS Pin to VDD
  'Read Ports from A-D
  READ_ADC1 = %1000_0000
  READ_ADC2 = %1001_0000
  READ_ADC3 = %1010_0000
  READ_ADC4 = %1011_0000
  READ_ADC5 = %1100_0000
  READ_ADC6 = %1101_0000
  READ_ADC7 = %1110_0000
  READ_ADC8 = %1111_0000
  'Accelerometer Address
  READ_ACCEL1_ADDR = %0011_1001
  WRITE_ACCEL1_ADDR = %0011_1000
  READ_ACCEL2_ADDR = %0011_1011
  WRITE_ACCEL2_ADDR = %0011_1010
  ACCEL_REGX = %0010_1001
  ACCEL_REGY = %0010_1011
  ACCEL_REGZ = %0010_1101
  'Super Compass!!!
  READ_COMPASS_ADDR =  %0011_0011
  WRITE_COMPASS_ADDR = %0011_0010
  COMPASS_ACCEL = %0100_0000
  COMPASS_MAG = %0100_0101
  COMPASS_HEADING = 0101_0000
  COMPASS_TILT = %0101_0101
  '$60 Compass
  READ_60_ADDR = %0100_0011
  WRITE_60_ADDR = %0100_0010
  MODE_60 = %0111_0010 '20Hz continuous with periodic set/reset


OBJ
  itsMotor  : "MotorControlPASM"
  itsI2C : "Basic_I2C_Driver"
  Computer : "CompComm_new"          'Computer Communications
  PID : "pid_new"
Var

  'ADC Address-Constants
  long READ_ADC[8]

  long accel_data[3]
  {{
    accel_data[0]=x status
    accel_data[1]=y status
    accel_data[2]=z status
  }}

  long adc_data[16]
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

  long desired_heading 'set by comp
  long desired_depth   'set by comp
  long desired_speed   'set by comp
  long marker_drop[2]
  'Compass Variables
  long comp_accel_data[3]
  {
    comp_accel_data[0] = x status
    comp_accel_data[1] = y status
    comp_accel_data[2] = z status
  }
  long comp_mag_data[3]
  {
   Mx, My, Mz
  }
  long comp_heading_data[3]
  {
   Head, Pitch, Roll
  }
  long comp_tilt_data[3]
  {
   Pitch, Roll, Temp
  }
  long opmode

  'PID Stuff
  long KK_depth
  long KK_heading
  long output_depth
  long output_heading
  long integral_time

  'PID Constants
  long kp_heading
  long kd_heading
  long ki_heading
  long kp_depth
  long kd_depth
  long ki_depth

  'Battery
  long battery1
  long battery2
  long battery3
  long battery4
  long compass_cal

  long enable[9]      'Enable for motors 0-5, Marker droppers 6-7, and aux motor 8
  long duty[9]        'Duty cycle for motors 0-5, Marker droppers 6-7, and aux motor 8
  long dir[9]         'Direction for motors 0-5, Marker droppers 6-7, and aux motor 8 
  long frequency
  long delay
  long pid_enableH
  long pid_enableD
  long depthGran
  long headingGran
  long killstatus
  long heartcount
  
  
Pub Main | motorsel, speedsel, freq, data, temp, channel

  pid_enableH := 0
  pid_enableD := 0
  depthGran := 1
  headingGran := 10
  desired_heading := 0 'set by comp
  desired_depth := 0   'set by comp
  desired_speed := 0   'set by comp
  'Setup ADC
  Set_ADC_Array

  'Start Computer Comm
  Computer.start(@accel_data)
  'Debug
  'Serial.start(31,30,0,115200)
  itsMotor.start(@enable)
  'Initialize PID
  initPID
  'Start PID
  pid.start(@accel_data)

 'Set Heartbeat
  dira[18] := 1
  outa[18] := 0
  heartcount := 0

  'Set up killstatus monitoring
  dira[19] := 0
  killstatus := 1
  
  'Set Accelerometer
  setupAccel

  'Set Compass
  'setupCompass
  'setupCompass60

  repeat
    'pid_enableD :=1
    heartcount += 1
    if heartcount==5
      !outa[18]
      heartcount := 0
    if compass_cal == $01
      enable[0]:=0
      enable[1]:=0
      enable[2]:=0
      enable[3]:=0
      enable[4]:=0
      enable[5]:=0
      StartCal
      compass_cal := $03
    elseif compass_cal == $03
      waitcnt(cnt+800_000)
    elseif compass_cal == $02
      EndCal
      compass_cal := $00
      enable[0]:=1
      enable[1]:=1
      enable[2]:=1
      enable[3]:=1
      enable[4]:=1
      enable[5]:=1      
    else
      if ina[19] == 1
        killstatus := 1
        itsMotor.Initialize_Motors
      else
        killstatus := 0
      readADC(0,6)
      readADC(0,7)
      'readAccel
      'resetCompass
      'runCompass
      'readCompass_Accel
      'readCompass_OpMode
      'readCompass_Mag
      'readCompass_Tilt
      'readCompass_Heading
      'read60Compass
      'checkCompass
      'standbyCompass
      'readBattery
      'debugPrint
    waitcnt(4_000_000+cnt)

'PUB readBattery

PUB read60Compass  | temp
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_COMPASS_ADDR)                  'Read slave command
    temp := 0
    temp := itsI2C.Read(I2C_SCL,0)
    temp <<= 8
    temp |= itsI2C.Read(I2C_SCL,1)
    comp_heading_data[0] := (~~temp)/10
    itsI2C.Stop(I2C_SCL)
PUB setupCompass60
  itsI2C.Start(I2C_SCL)
  itsI2C.Write(I2C_SCL, WRITE_60_ADDR)
  itsI2C.Write(I2C_SCL, %0100_0111)                     'Write to RAM Register
  itsI2C.Write(I2C_SCL, %0111_0100)                     'Op Register
  itsI2C.Write(I2C_SCL, MODE_60)                     'Mode
  itsI2C.Stop(I2C_SCL)

PUB CheckCompass | count
  count := 0
  repeat while (comp_heading_data[0] <0 OR comp_heading_data[0] >3600) AND count < 5
    readCompass_Heading
    count++
  if count == 5
    comp_heading_data[0] := -1

PUB initPID
  KK_heading := 0
  KK_depth := 0
  KP_heading := 1
  KD_heading := 0
  KI_heading := 0
  KP_depth := 1
  KD_depth := 0
  KI_depth := 0
  integral_time := 1 'ms
  desired_heading := 0
  desired_depth := 0

  

PUB StartCal
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)
    itsI2C.Write(I2C_SCL, $71)
    itsI2C.Stop(I2C_SCL)

PUB EndCal
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)
    itsI2C.Write(I2C_SCL, $7E)
    itsI2C.Stop(I2C_SCL)
        
PUB resetCompass
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)
    itsI2C.Write(I2C_SCL, $82)
    itsI2C.Stop(I2C_SCL)
    waitcnt(50_000_000+cnt)

PUB runCompass
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)
    itsI2C.Write(I2C_SCL, $75)
    itsI2C.Stop(I2C_SCL)
    waitcnt(80_000+cnt)

PUB standbyCompass
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)
    itsI2C.Write(I2C_SCL, $76)
    itsI2C.Stop(I2C_SCL)
    waitcnt(80_000+cnt)

PUB readCompass_OpMode
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)                  'Write to slave command
    itsI2C.Write(I2C_SCL, $E1)
    itsI2C.Write(I2C_SCL, $05)
    waitcnt(100_000+cnt)
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_COMPASS_ADDR)                  'Read from slave command
    opmode := itsI2C.Read(I2C_SCL,1)
    itsI2c.Stop(I2C_SCL)
PUB readCompass_Accel| temp,n
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)                  'Write to slave command
    itsI2C.Write(I2C_SCL, COMPASS_ACCEL)
    waitcnt(100_000+cnt)
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_COMPASS_ADDR)                  'Read from slave command
    repeat n from 0 to 2
      temp := 0
      temp := itsI2C.Read(I2C_SCL,0)
      temp <<= 8
      if n == 2
        temp |= itsI2C.Read(I2C_SCL,1)
      else
        temp |= itsI2C.Read(I2C_SCL,0)
      comp_accel_data[n] := ~~temp
    itsI2C.Stop(I2C_SCL)

PUB readCompass_Mag| temp,n
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)                  'Write to slave command
    itsI2C.Write(I2C_SCL, COMPASS_MAG)
    waitcnt(100_000+cnt)
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_COMPASS_ADDR)                  'Read from slave command
    repeat n from 0 to 2
      temp := 0
      temp := itsI2C.Read(I2C_SCL,0)
      temp <<= 8
      if n == 2
        temp |= itsI2C.Read(I2C_SCL,1)
      else
        temp |= itsI2C.Read(I2C_SCL,0)
      comp_mag_data[n] := ~~temp
    itsI2C.Stop(I2C_SCL)

PUB readCompass_Heading| temp,n
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)                  'Write to slave command
    itsI2C.Write(I2C_SCL, COMPASS_HEADING)
    waitcnt(160_000+cnt)
    'waitcnt(8_000_000+cnt)
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_COMPASS_ADDR)                  'Read from slave command
    temp := 0
    temp := itsI2C.Read(I2C_SCL,0)
    temp <<= 8
    temp |= itsI2C.Read(I2C_SCL,1)
    {{    
    repeat n from 0 to 2
      temp := 0
      temp := itsI2C.Read(I2C_SCL,0)
      temp <<= 8
      if n == 2
        temp |= itsI2C.Read(I2C_SCL,1)
      else
        temp |= itsI2C.Read(I2C_SCL,0)
        
      comp_heading_data[n] := ~~temp
      }}
    comp_heading_data[0] := (~~temp)/10
    itsI2C.Stop(I2C_SCL)
    

PUB readCompass_Tilt| temp,n
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)                  'Write to slave command
    itsI2C.Write(I2C_SCL, COMPASS_TILT)
    waitcnt(100_000+cnt)
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_COMPASS_ADDR)                  'Read from slave command
    repeat n from 0 to 2
      temp := 0
      temp := itsI2C.Read(I2C_SCL,0)
      temp <<= 8
      if n == 2
        temp |= itsI2C.Read(I2C_SCL,1)
      else
        temp |= itsI2C.Read(I2C_SCL,0)
      comp_tilt_data[n] := ~~temp
    itsI2C.Stop(I2C_SCL)

PUB Set_ADC_Array
  READ_ADC[0]:=READ_ADC1
  READ_ADC[1]:=READ_ADC2
  READ_ADC[2]:=READ_ADC3
  READ_ADC[3]:=READ_ADC4
  READ_ADC[4]:=READ_ADC5
  READ_ADC[5]:=READ_ADC6
  READ_ADC[6]:=READ_ADC7
  READ_ADC[7]:=READ_ADC8

PUB readAccel | temp
    temp := 0
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_ACCEL1_ADDR)                  'Write to slave command
    itsI2C.Write(I2C_SCL, ACCEL_REGX)
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_ACCEL1_ADDR)                  'Read from slave command
    temp := itsI2C.Read(I2C_SCL,1)
    accel_data[0] := ~temp
    itsI2C.Stop(I2C_SCL)

    temp := 0
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_ACCEL1_ADDR)                  'Write to slave command
    itsI2C.Write(I2C_SCL, ACCEL_REGY)
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_ACCEL1_ADDR)                  'Read from slave command
    temp := itsI2C.Read(I2C_SCL,1)
    accel_data[1] := ~temp
    itsI2C.Stop(I2C_SCL)

    temp := 0
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, WRITE_ACCEL1_ADDR)                  'Write to slave command
    itsI2C.Write(I2C_SCL, ACCEL_REGZ)
    itsI2C.Start(I2C_SCL)
    itsI2C.Write(I2C_SCL, READ_ACCEL1_ADDR)                  'Read from slave command
    temp := itsI2C.Read(I2C_SCL,1)
    accel_data[2] := ~temp
    itsI2C.Stop(I2C_SCL)

PUB readADC(chip,channel)| temp
    'repeat channel from 0 to 7
    if chip == 0
      itsI2C.Initialize(I2C_SCL)
      temp:=0
      itsI2C.Start(I2C_SCL)
      itsI2C.Write(I2C_SCL,ADC1_ADDR | WRITE)                  'Write to slave command
      itsI2C.Write(I2C_SCL,READ_ADC[channel])
      itsI2C.Start(I2C_SCL)
      itsI2C.Write(I2C_SCL,ADC1_ADDR | READ)                  'Read from slave command
      temp := itsI2C.Read(I2C_SCL,0)
      temp <<= 8
      temp := temp | itsI2C.Read(I2C_SCL,1)
      temp &= %0000_1111_1111_1111
      adc_data[channel]:= temp
    'repeat channel from 0 to 7
    else
      itsI2C.Initialize(I2C_SCL)
      temp:=0
      itsI2C.Start(I2C_SCL)
      itsI2C.Write(I2C_SCL,ADC2_ADDR | WRITE)                  'Write to slave command
      itsI2C.Write(I2C_SCL,READ_ADC[channel])
      itsI2C.Start(I2C_SCL)
      itsI2C.Write(I2C_SCL,ADC2_ADDR | READ)                  'Read from slave command
      temp := itsI2C.Read(I2C_SCL,0)
      temp <<= 8
      temp := temp | itsI2C.Read(I2C_SCL,1)
      temp &= %0000_1111_1111_1111
      adc_data[channel+8]:= temp

PUB setupAccel
  itsI2C.Start(I2C_SCL)
  itsI2C.Write(I2C_SCL, WRITE_ACCEL1_ADDR)
  itsI2C.Write(I2C_SCL, %0010_0000)                     'CTRL_REG1
  itsI2C.Write(I2C_SCL, %1100_0111)                     '400Hz,
  itsI2C.Stop(I2C_SCL)

PUB setupCompass
  itsI2C.Start(I2C_SCL)
  itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)
  itsI2C.Write(I2C_SCL, %1111_0001)                     'Write to EEPROM
  itsI2C.Write(I2C_SCL, %0000_0101)                     'Op Mode 2 Reg
  itsI2C.Write(I2C_SCL, %0000_0010)                     '10Hz
  itsI2C.Stop(I2C_SCL)

  itsI2C.Start(I2C_SCL)
  itsI2C.Write(I2C_SCL, WRITE_COMPASS_ADDR)
  itsI2C.Write(I2C_SCL, %0111_0010)                     'Level Orientation
  itsI2C.Stop(I2C_SCL)

CON
  PIN_M0_PWM = 6                
  PIN_M0_DIR = 7
  PIN_M1_PWM = 15
  PIN_M1_DIR = 5
  PIN_M2_PWM = 13
  PIN_M2_DIR = 14
  PIN_M3_PWM = 1
  PIN_M3_DIR = 0
  PIN_M4_PWM = 10
  PIN_M4_DIR = 2
  PIN_M5_PWM = 8
  PIN_M5_DIR = 9
  PIN_M6_PWM = 11
  PIN_M6_DIR = 12
  PIN_M7_PWM = 16
  PIN_M7_DIR = 17
  PIN_M8_PWM = 3
  PIN_M8_DIR = 4
  
VAR
  long pwmpin[9]      'Array holding pwm pin#
  long dirpin[9]      'Array holding direction pin#
  long enablep[9]      'Enable for motors 0-5, Marker droppers 6-7, and aux motor 8
  long dutyp[9]        'Duty cycle for motors 0-5, Marker droppers 6-7, and aux motor 8
  long dirp[9]         'Direction for motors 0-5, Marker droppers 6-7, and aux motor 8 
  long frequencyp
  long delayp
  long motori         'Counter
  long motorCog     

PUB start(address)
  setAddress(address)
  Set_Pin_Arrays                'stores Pin numbers into easily accessible arrays
  'Set PWM frequency
  long[frequencyp] := 250
  long[delayp] := CLKFREQ/long[frequencyp]/100  
  Initialize_Motors
  motorCog := cognew(@init,address) 

PUB setAddress(addr)
enablep[0] := addr+4*0
enablep[1] := addr+4*1
enablep[2] := addr+4*2
enablep[3] := addr+4*3
enablep[4] := addr+4*4
enablep[5] := addr+4*5
enablep[6] := addr+4*6
enablep[7] := addr+4*7
enablep[8] := addr+4*8
dutyp[0] := addr+4*9
dutyp[1] := addr+4*10
dutyp[2] := addr+4*11
dutyp[3] := addr+4*12
dutyp[4] := addr+4*13
dutyp[5] := addr+4*14
dutyp[6] := addr+4*15
dutyp[7] := addr+4*16
dutyp[8] := addr+4*17
dirp[0] := addr+4*18  
dirp[1] := addr+4*19
dirp[2] := addr+4*20
dirp[3] := addr+4*21
dirp[4] := addr+4*22
dirp[5] := addr+4*23
dirp[6] := addr+4*24
dirp[7] := addr+4*25
dirp[8] := addr+4*26
frequencyp := addr+4*27
delayp := addr+4*28
DAT
                        org     0
                        'Get address pointers
init                    mov     enableptr0, par
                        mov     enableptr1, enableptr0
                        add     enableptr1, #4
                        mov     enableptr2, enableptr1
                        add     enableptr2, #4
                        mov     enableptr3, enableptr2
                        add     enableptr3, #4       
                        mov     enableptr4, enableptr3
                        add     enableptr4, #4
                        mov     enableptr5, enableptr4
                        add     enableptr5, #4
                        mov     enableptr6, enableptr5
                        add     enableptr6, #4
                        mov     enableptr7, enableptr6
                        add     enableptr7, #4
                        mov     enableptr8, enableptr7
                        add     enableptr8, #4
                        mov     dutyptr0, enableptr8
                        add     dutyptr0, #4
                        mov     dutyptr1, dutyptr0
                        add     dutyptr1, #4
                        mov     dutyptr2, dutyptr1
                        add     dutyptr2, #4
                        mov     dutyptr3, dutyptr2
                        add     dutyptr3, #4
                        mov     dutyptr4, dutyptr3
                        add     dutyptr4, #4
                        mov     dutyptr5, dutyptr4
                        add     dutyptr5, #4
                        mov     dutyptr6, dutyptr5
                        add     dutyptr6, #4
                        mov     dutyptr7, dutyptr6
                        add     dutyptr7, #4
                        mov     dutyptr8, dutyptr7
                        add     dutyptr8, #4                                                                                                                                                
                        mov     dirptr0, dutyptr8
                        add     dirptr0, #4
                        mov     dirptr1, dirptr0
                        add     dirptr1, #4
                        mov     dirptr2, dirptr1
                        add     dirptr2, #4
                        mov     dirptr3, dirptr2
                        add     dirptr3, #4
                        mov     dirptr4, dirptr3
                        add     dirptr4, #4
                        mov     dirptr5, dirptr4
                        add     dirptr5, #4
                        mov     dirptr6, dirptr5
                        add     dirptr6, #4
                        mov     dirptr7, dirptr6
                        add     dirptr7, #4
                        mov     dirptr8, dirptr7
                        add     dirptr8, #4
                        mov     freqptr, dirptr8
                        add     freqptr, #4
                        mov     delayptr, freqptr
                        add     delayptr, #4
                        
                        'Set pins to output
                        or      dira, ENABLE_MASK
                        'Set pins high
                        mov     outa, ENABLE_MASK                       
                        'Get Output Pin Status
                        mov     tempout, outa 
                        'Setup Initial Time
                        mov     time, cnt
                        rdlong  loopdelay, delayptr
                        add     time, loopdelay
                        mov     cycle, #99               'cycle := 0
                        'Begin Loop
pwm_loop                waitcnt time, loopdelay                                           
                        'Increment cycle
                        add     cycle, #1               'cycle += 1
                        cmp     cycle, #100 wz          'if(cycle == 100)
              if_z      jmp     #restart_pwm            'restart cycle, reset PWM
                        'Check PWM to flip
check_pwm               cmp     cycle, duty0 wz          'if(cycle == duty[0])
              if_z      or      tempout, M0_PWM         ' M0_PWM := 1
                        cmp     cycle, duty1 wz          'if(cycle == duty[1])
              if_z      or      tempout, M1_PWM         ' M1_PWM := 1
                        cmp     cycle, duty2 wz          'if(cycle == duty[2])
              if_z      or      tempout, M2_PWM         ' M2_PWM := 1
                        cmp     cycle, duty3 wz          'if(cycle == duty[3])
              if_z      or      tempout, M3_PWM         ' M3_PWM := 1
                        cmp     cycle, duty4 wz          'if(cycle == duty[4])
              if_z      or      tempout, M4_PWM         ' M4_PWM := 1
                        cmp     cycle, duty5 wz          'if(cycle == duty[5])
              if_z      or      tempout, M5_PWM         ' M5_PWM := 1
                        cmp     cycle, duty6 wz          'if(cycle == duty[6])
              if_z      or      tempout, M6_PWM         ' M6_PWM := 1
                        cmp     cycle, duty7 wz          'if(cycle == duty[7])
              if_z      or      tempout, M7_PWM         ' M7_PWM := 1
                        cmp     cycle, duty8 wz          'if(cycle == duty[8])
              if_z      or      tempout, M8_PWM         ' M8_PWM := 1
                        'Write to Output
                        mov     outa, tempout
                        'End Loop
                        jmp     #pwm_loop                        
'==============================================================================              
restart_pwm             mov     cycle, #0               'cycle := 0
                        andn    tempout, PWM_HIGH       'Set PWM pins low
                        andn    tempout, DIR_MASK       'Set DIR pins low
                        rdlong  loopdelay, delayptr     'Set new Frequency delay
                        
check_dir               'Check Direction Pin 0
                        rdlong  temp, dirptr0           'temp := dir[0]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M0_DIR         'Set dir pin
                        'Direction Pin 1
                        rdlong  temp, dirptr1           'temp := dir[1]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M1_DIR         'Set dir pin
                        'Direction Pin 2
                        rdlong  temp, dirptr2           'temp := dir[2]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M2_DIR         'Set dir pin
                        'Direction Pin 3
                        rdlong  temp, dirptr3           'temp := dir[3]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M3_DIR         'Set dir pin
                        'Direction Pin 4
                        rdlong  temp, dirptr4           'temp := dir[4]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M4_DIR         'Set dir pin
                        'Direction Pin 5
                        rdlong  temp, dirptr5           'temp := dir[5]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M5_DIR         'Set dir pin
                        'Direction Pin 6
                        rdlong  temp, dirptr6           'temp := dir[6]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M6_DIR         'Set dir pin
                        'Direction Pin 7
                        rdlong  temp, dirptr7           'temp := dir[7]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M7_DIR         'Set dir pin
                        'Direction Pin 8
                        rdlong  temp, dirptr8           'temp := dir[8]
                        cmp     temp, #0 wz             'if(dir[0] != 0)
              if_nz     or      tempout, M8_DIR         'Set dir pin

get_new_PWM             'PWM0
                        rdlong  duty0, dutyptr0
                        'PWM1
                        rdlong  duty1, dutyptr1
                        'PWM2
                        rdlong  duty2, dutyptr2
                        'PWM3
                        rdlong  duty3, dutyptr3
                        'PWM4
                        rdlong  duty4, dutyptr4
                        'PWM5
                        rdlong  duty5, dutyptr5
                        'PWM6
                        rdlong  duty6, dutyptr6
                        'PWM7
                        rdlong  duty7, dutyptr7
                        'PWM8
                        rdlong  duty8, dutyptr8

check_enable            'If disabled, set pwm pin high
                        'Enable 0
                        rdlong  temp, enableptr0
                        cmp     temp, #0 wz             'if (enable[0] == 0)
              if_z      or      tempout,M0_PWM          'outa[PIN_M0_PWM] := 1 
                        'Enable 1
                        rdlong  temp, enableptr1
                        cmp     temp, #0 wz             'if (enable[1] == 0)
              if_z      or      tempout,M1_PWM          'outa[PIN_M1_PWM] := 1  
                        'Enable 2
                        rdlong  temp, enableptr2
                        cmp     temp, #0 wz             'if (enable[2] == 0)
              if_z      or      tempout,M2_PWM          'outa[PIN_M2_PWM] := 1 
                        'Enable 3
                        rdlong  temp, enableptr3
                        cmp     temp, #0 wz             'if (enable[3] == 0)
              if_z      or      tempout,M3_PWM          'outa[PIN_M3_PWM] := 1
                        'Enable 4
                        rdlong  temp, enableptr4
                        cmp     temp, #0 wz             'if (enable[4] == 0)
              if_z      or      tempout,M4_PWM          'outa[PIN_M4_PWM] := 1
                        'Enable 5
                        rdlong  temp, enableptr5
                        cmp     temp, #0 wz             'if (enable[5] == 0)
              if_z      or      tempout,M5_PWM          'outa[PIN_M5_PWM] := 1
                        'Enable 6
                        rdlong  temp, enableptr6
                        cmp     temp, #0 wz             'if (enable[6] == 0)
              if_z      or      tempout,M6_PWM          'outa[PIN_M6_PWM] := 1
                        'Enable 7
                        rdlong  temp, enableptr7
                        cmp     temp, #0 wz             'if (enable[7] == 0)
              if_z      or      tempout,M7_PWM          'outa[PIN_M7_PWM] := 1
                        'Enable 8
                        rdlong  temp, enableptr8
                        cmp     temp, #0 wz             'if (enable[8] == 0)
              if_z      or      tempout,M8_PWM          'outa[PIN_M8_PWM] := 1
              
                        jmp     #check_pwm   
                         
                                                                                                                                                                      

'Set Pin Numbers
PWM_HIGH      long      1<<6 | 1<<15 | 1<<13 | 1<<1 | 1<<10 | 1<<8 | 1<<11 | 1<<16 | 1<<3
DIR_MASK      long      1<<7 | 1<<5 | 1<<14 | 1 | 1<<2 | 1<<9 | 1<<12 | 1<<17 | 1<<4
ENABLE_MASK   long      1<<6 | 1<<15 | 1<<13 | 1<<1 | 1<<10 | 1<<8 | 1<<11 | 1<<16 | 1<<3 | 1<<7 | 1<<5 | 1<<14 | 1 | 1<<2 | 1<<9 | 1<<12 | 1<<17 | 1<<4
M0_PWM        long      1<<6                
M0_DIR        long      1<<7
M1_PWM        long      1<<15
M1_DIR        long      1<<5
M2_PWM        long      1<<13
M2_DIR        long      1<<14
M3_PWM        long      1<<1
M3_DIR        long      1
M4_PWM        long      1<<10
M4_DIR        long      1<<2
M5_PWM        long      1<<8
M5_DIR        long      1<<9
M6_PWM        long      1<<11
M6_DIR        long      1<<12
M7_PWM        long      1<<16
M7_DIR        long      1<<17
M8_PWM        long      1<<3
M8_DIR        long      1<<4
cycle         long      0

'Uninitialized Variables
enableptr0    res       1
enableptr1    res       1
enableptr2    res       1
enableptr3    res       1
enableptr4    res       1
enableptr5    res       1
enableptr6    res       1
enableptr7    res       1
enableptr8    res       1
dutyptr0      res       1
dutyptr1      res       1
dutyptr2      res       1
dutyptr3      res       1
dutyptr4      res       1
dutyptr5      res       1
dutyptr6      res       1
dutyptr7      res       1
dutyptr8      res       1
dirptr0       res       1
dirptr1       res       1
dirptr2       res       1
dirptr3       res       1
dirptr4       res       1
dirptr5       res       1
dirptr6       res       1
dirptr7       res       1
dirptr8       res       1
duty0         res       1
duty1         res       1
duty2         res       1
duty3         res       1
duty4         res       1
duty5         res       1
duty6         res       1
duty7         res       1
duty8         res       1
time          res       1
freqptr       res       1
delayptr      res       1
loopdelay     res       1
temp          res       1
tempout       res       1
dirout        res       1               

Pub setMotorStatus(motor, status)
'Enable/Disable motor (1/0)
  long[enablep[motor]] := status  

Pub setFreq(newFreq) 'newFreq in unit of 10Hz, range from 0-2550Hz
  long[frequencyp] := newFreq*10
  long[delayp] := CLKFREQ/long[frequencyp]/100
                                                 
PUB Set_Pin_Arrays              'Store all pins numbers into two easily accessible arrays
  pwmpin[0]:=PIN_M0_PWM
  pwmpin[1]:=PIN_M1_PWM
  pwmpin[2]:=PIN_M2_PWM
  pwmpin[3]:=PIN_M3_PWM
  pwmpin[4]:=PIN_M4_PWM
  pwmpin[5]:=PIN_M5_PWM 
  pwmpin[6]:=PIN_M6_PWM
  pwmpin[7]:=PIN_M7_PWM
  pwmpin[8]:=PIN_M8_PWM
  dirpin[0]:=PIN_M0_DIR
  dirpin[1]:=PIN_M1_DIR
  dirpin[2]:=PIN_M2_DIR
  dirpin[3]:=PIN_M3_DIR
  dirpin[4]:=PIN_M4_DIR
  dirpin[5]:=PIN_M5_DIR 
  dirpin[6]:=PIN_M6_DIR
  dirpin[7]:=PIN_M7_DIR
  dirpin[8]:=PIN_M8_DIR  

PUB Initialize_Motors           'Set all pins to forward at 0%
  repeat motori from 0 to 8 
    'OUTA[dirpin[motori]]:=1
    'OUTA[pwmpin[motori]]:=0     
    long[dirp[motori]] := 1            'Motor direction
    long[dutyp[motori]] := 0                              
    'long[dutyp[motori]] := motori*10
    long[enablep[motori]]:=1           'Input/Output direction

PUB Stop
''Stops the Cog and the PID controller
cogstop(motorCog)      
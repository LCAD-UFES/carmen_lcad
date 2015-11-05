{
 ************************************************************************************************************
 *                                                                                                          *
 *  AUTO-RECOVER NOTICE: This file was automatically recovered from an earlier Propeller Tool session.      *
 *                                                                                                          *
 *  ORIGINAL FOLDER:     \\java.usc.edu\java\Folder-Redirect\beobot\Desktop\robotarm\                       *
 *  TIME AUTO-SAVED:     1 hour, 30 minutes ago (4/26/2007 1:00:08 PM)                                      *
 *                                                                                                          *
 *  OPTIONS:             1)  RESTORE THIS FILE by deleting these comments and selecting File -> Save.       *
 *                           The existing file in the original folder will be replaced by this one.         *
 *                                                                                                          *
 *                           -- OR --                                                                       *
 *                                                                                                          *
 *                       2)  IGNORE THIS FILE by closing it without saving.                                 *
 *                           This file will be discarded and the original will be left intact.              *
 *                                                                                                          *
 ************************************************************************************************************
.}
{{ 
     L298MotorsPwm.spin
     Lior Elazary
     Obtained from: Tom Doyle
     25 April 2007

     Starts a cog to maintain a PWM signal to the L298 chip
     The control pins on the L298 are controlled by the forward and reverse procedures
     Speed is controlled by the update procedure

     In normal use it is not necessary to call any of these procedures directly as
     they are called by the SetMotor.spin object

     Enable pin is no longer used, only 2 pins are used. The pwm is assigen
     to the current pin based on direction
}} 

CON

 _clkmode = xtal1 + pll16x
 _xinfreq = 5_000_000
 

VAR

  long duty, period, dirM, I1pin, I2Pin, P0pin , P1pin ,EncoderVal, desEncoderVal, isPID, err, Kp, Ki, Kd, speedOffset' par access
  
  byte cog

PUB start(P0,P1,In1Pin, In2Pin, pulsesPerCycle) : success

    ' P0 - Motor Encoder 1
    ' P1 - Motor Encoder 2
    ' In1Pin - L298 Input 1 Pin
    ' In2Pin - L298 Input 2 Pin
    ' pulsesPerCycle - pulses per PWM cycle = clkfreq/pwmfreq

    P0pin := P0
    P1Pin := P1
    I1pin  := In1Pin
    I2pin  := In2Pin

    desEncoderVal := 0
    isPID := 0
    EncoderVal := 0

    duty   := 0   'Reverse, 100 is off
    period := pulsesPerCycle
    
    reverse                                           ' initialize dirM
    success   := cog := cognew(@entry, @duty)
    update(0) 

PUB stop
{{ set esc PWM pin to off
   stop cog }}
   
    'waitpeq(0, |< pPin, 0)
    dira[I1pin] := 0
    dira[I2pin] := 0  
    if cog > 0
      cogstop(cog)

PUB forward

    dirM := !0

Pub reverse

    dirM := 0


PUB update(dutyPercent)

    duty := period * dutyPercent / 100

PUB readEncoder: Value
  value := EncoderVal
  
PUB resetEncoder
  EncoderVal := 0

PUB getSpeed: val
  val := duty '(duty/period)*100
  
PUB setPosition(pos)
  desEncoderVal := pos

PUB setPIDCtrl(val)
  isPID := val

PUB getPosition: pos
  pos := desEncoderVal

PUB getErr: val
  val := err

PUB setKp(val)
  Kp := val

PUB setKi(val)
  Ki := val

PUB setKd(val)
  Kd := val 

PUB setSpeedOffset(val)
  speedOffset := val
  

DAT

entry                   movi   ctra,#%00100_000   'set timer a to pwm
                        movd   ctra,#0

                        mov     addr, par        
                        add     addr, #28        ' Encoder val
                        mov     _enVal, addr     ' stored addr in _enVal

                       mov     mask, #$FF    'build mask
                       shl     mask, #8
                       add     mask, #$FF


' pid params
                        mov     addr, par         'speed offset
                        add     addr, #56
                        rdlong  _speedOffset, addr

                        
                        mov     addr, par         'kp
                        add     addr, #44
                        rdlong  _Kp, addr

                        mov     addr, par         'ki
                        add     addr, #48
                        rdlong  _Ki, addr

                        mov     addr, par         'kd
                        add     addr, #52
                        rdlong  _Kd, addr


                        mov     addr, par                                                                   
                        add     addr, #8          ' dirM
                        mov     _ptrDirM, addr

                        mov     addr, par       
                        add     addr, #1
                        mov     _ptrDuty, addr

                        mov     addr, par       
                        add     addr, #40
                        mov     _ptrErr, addr

                        mov     _isPID, #0

                        mov     _speed, #0
                        mov     _iErr, #0
                        

'encoder pins
                        mov     addr, par        
                        add     addr, #20        ' Encoder P0 pin
                        rdlong  _P0Pin, addr     ' stored in _p0pin

                        mov     _ap, _P0Pin
                        mov     temp, #1
                        shl     temp,_P0Pin     ' L298 In1 pin
                        mov     _P0Pin, temp
                        
                        mov     addr, par        
                        add     addr, #24        ' Encoder p1 pin
                        rdlong  _P1Pin, addr     ' stored in _p1pin

                        mov     _bp, _P1Pin
                        mov     temp, #1
                        shl     temp,_P1Pin     ' L298 In1 pin
                        mov     _P1Pin, temp



                        

'pwm pins                  
                        mov     addr, par        
                        add     addr, #12        ' L298 In1 pin
                        rdlong  _In1Pin, addr    ' stored in _In1Pin

                        mov     temp, #1
                        shl     temp,_In1pin     ' L298 In1 pin
                        or      dira, temp       ' make an output

                        mov     addr, par        
                        add     addr, #16        ' L298 In2 pin
                        rdlong  _In2Pin, addr    ' stored in _In2Pin

                        mov     temp, #1
                        shl     temp,_In2pin     ' L298 In2 pin
                        or      dira, temp       ' make an output


                       

                        'get initial state for encoders
                        mov     _count, #0   
                        mov     _a, ina          'read encoders
                        mov     _b, _a           'copy result so have full port in _a and _b 
                        and     _a, _P0Pin       'mask (now only have state of pin A)
                        shr     _a, _ap          'shift bits (puts that state in bit 0)
                        and     _b, _P1Pin       'mask   
                        shr     _b, _bp          'shift again                        
                        shl     _b, #1           'shift back one to put in bit 1                          
                        or      _a,_b            '_a now holds the encoder reading for both channels
                        mov     _old,_a          'this is the "old" state
                        mov     _state,_a        'loaded into state variable
                                        
                        mov     frqa,#1

                        mov     addr, par
                        add     addr, #4         ' pulses per pwm cycle
                        rdlong  _cntadd, addr                        
                        mov     cntacc,cnt
                        add     cntacc,_cntadd                       

:loop     

                       
                       
'pwm
                        mov     temp, cnt
                        cmp     temp, cntacc     WC   'c is set if cnt < cntacc 
              if_c      jmp     #:encoder_calc

                        mov     addr, par        
                        add     addr, #36        ' Whether to preform pid control
                        rdlong  _isPID, addr     ' stored value in _isPID

                        
                        tjz     _isPID, #:userPwmData     'check if to preform pid control
'get values from pid
                       'set PID params
                        mov     addr, par        
                        add     addr, #32        ' Desired encoder val
                        rdlong  _desEnVal, addr     ' stored value in _desEnVal


         
'                        jmp     #:pwmDataDone 

:userPwmData
'get values from params
                        rdlong  _dirM, _ptrDirM        ' store in _dirM
                        rdlong  _duty, _ptrDuty
:pwmDataDone                        


                        mov     tempDir, outa
                        tjz     _dirM, #:In2Pwm     'check the direction
                        movs    ctra,_In1Pin       'pwm on pin1
                        mov     temp, #1
                        shl     temp, _In2Pin
                        jmp     #:PinPwmDone
:In2Pwm                 movs    ctra ,_In2Pin      'pwm on pin2
                        mov     temp, #1
                        shl     temp, _In1Pin

:PinPwmDone
                        mov     outa, tempDir
                        neg     phsa,_duty

                        add     cntacc, _cntadd    'wait for the next count

'read and encode encoder values
:encoder_calc
                        mov     _a, ina          'read encoders
                        mov     _b, _a           'copy result
                        and     _a, _P0Pin       'mask
                        shr     _a, _ap          'shift bits
                        and     _b, _P1Pin       'mask
                        shr     _b, _bp          'shift again
                        shl     _b, #1           'shift back one to make bit 1
                        or      _a,_b            '_a now holds the encoder reading
                        
                        cmp     _old,_a          wz
                if_z    jmp     #:PIDCalc           'if no change has occured loop



                        'if Change has occured                        
                        shl     _state, #2       'shift previous reading into bits 2,3
                        or      _state, _a       'put current value in bits 0,1
                        mov     _old, _a         'record current as new old :)                        
                        and     _a,#0            'clear a and b
                        and     _b,#0            '
       
                        'Decide what to do 
                        and     _state,#7        'only need first three bits for direction identify
 
                        cmp     _state,#2       wz 'is state 010?
              if_z      add     _count,#1          'if yes increment count and loop
              if_z      jmp     #:write_enVal       
                        cmp     _state,#3       wz 'is state 011?
              if_z      add     _count,#1
              if_z      jmp     #:write_enVal
                        cmp     _state,#4       wz 'etc
              if_z      add     _count,#1
              if_z      jmp     #:write_enVal
                        cmp     _state,#5       wz
              if_z      add     _count,#1
              if_z      jmp     #:write_enVal
                        sub     _count,#1           'otherwise must be going other way 
                        jmp     #:write_enVal              'and loop 

:write_enVal
                        wrlong  _count, _enVal

                    
:PIDCalc
'PID                    
                        tjz     _isPID, #:loop     'check if to preform pid control

                        mov     _err, _desEnVal
                        subs    _err, _count       'err = desEnVal - count

                        add     _iErr, _err      'int err
'                        wrlong  _err, _ptrErr                           
                        
                        'P gain
                        mov     t1, _err
                        mov     t2, _Kp
                        call    #multiply       
                        mov     _speed, t1      '_speed = _Kp*_err


                        'I gain
                        mov     t1,_iErr
                        mov     t2,_Ki
                        call    #divide
                        add     _speed, t1      '_speed = _speed + _Ki*_iErr
                         
        
'                        add     _speed, temp
                        wrlong  _iErr, _ptrErr

                        'D gain
'                        mov     temp, _oldErr
'                        subs    temp, _err
'                        sar     temp, #20
'                        add     _speed, temp 

                         mov     _oldErr, _err
                         'set the direction


                         
                         cmp    _err, #0   WZ
              if_nz      jmp    #:setSpeed    'if err is not 0


                         'set the speed to 0
'                         mov _duty, #0
'                         mov _iErr, #0
                         wrlong  _duty, _ptrDuty
                         jmp #:loop
           

:setSpeed                                               
                         cmps     _speed, #0     WC   'c is set if speed < 0
              if_c       jmp     #:PID_reverse
                         mov    _dirM, #1
                         jmp     #:end_PID_dir  
:PID_reverse
                         mov   _dirM, #0
:end_PID_dir 


                         abs     _duty, _speed
        '                cmp     _duty, _speedOffset    WC  'c is set if duty < _speedOffset
        '     if_nc      jmp     #:noSpeedOffset
        '                mov     _duty, _speedOffset     

                        
:noSpeedOffset
                        wrlong  _dirM, _ptrDirM
                        wrlong  _duty, _ptrDuty

                        
                        jmp     #:loop

' Multiply
'
'   in:         t1 = 16-bit multiplicand (t1[31..16] must be 0)
'               t2 = 16-bit multiplier
'
'   out:        t1 = 32-bit product
'
multiply
                        mov     t4, t1
                        xor     t4, t2          'bit 31 = 1 neg
                        
                        abs     t1, t1          'take the abs of the numbers
                        abs     t2, t2
                        
                        mov     t3,#16          'multiply by shifing it 16 times (can be faster if unrolled)
                        shl     t2,#16
                        shr     t1,#1           wc

:mulloop   if_c         add     t1,t2           wc
                        rcr     t1,#1           wc
                        djnz    t3,#:mulloop

                        'correct the sign
                        shl     t4, #1          wc
           if_c         neg     t1, t1        

multiply_ret            ret

'
' Divide t1[31..0] by t2[15..0] (t2[16] must be 0)
' on exit, quotient is in t1[15..0] and remainder is in t1[31..16]
'
divide                  mov     t4, t1
                        xor     t4, t2          'bit 31 = 1 neg
                        
                        abs     t1, t1          'take the abs of the numbers
                        abs     t2, t2
                        
                        shl t2,#15                'get divisor into y[30..15]
                        mov t3,#16                'ready for 16 quotient bits
                        
:loop                   cmpsub t1,t2 wc            'if y =< x then subtract it, quotient bit into c
                        rcl t1,#1                 'rotate c into quotient, shift dividend
                        djnz t3,#:loop            'loop until done

                        and    t1, mask 
                        'correct the sign
                        shl     t4, #1          wc
           if_c         neg     t1, t1
           
divide_ret              ret                      'quotient in x[15..0], remainder in x[31..16]


_dirM                   res     1    ' motor direction 0 or 1
_ptrDirM                res     1
_In1Pin                 res     1    ' L298 Input 1 Pin
_In2Pin                 res     1    ' L298 Input 2 Pin
tempDir                 res     1    ' temp direction
cntacc                  res     1
_duty                   res     1
_ptrDuty                res     1
_cntadd                 res     1
_Enpin                  res     1     ' L298 Enable Pin
_p0pin                  res     1     ' Encoder P0
_p1pin                  res     1     ' Encoder P1
_enVal                  res     1     ' Encoder Value Addr
_desEnVal               res     1      ' desired encoder pos
_isPID                  res     1 
_Kp                     res     1     ' pid params
_Ki                     res     1
_Kd                     res     1
_err                    res     1
_iErr                   res     1
_oldErr                 res     1
_ptrErr                 res     1
_speed                  res     1
_speedOffset            res     1
            
addr                    res     1
temp                    res     1
_count                  res     1
_a_pin                  res     1           'This is the A channel pin
_b_pin                  res     1           'This is the B channel pin
_a                      res     1
_b                      res     1
_state                  res     1
_old                    res     1
_ap                     res     1
_bp                     res     1
t1                      res     1
t2                      res     1
t3                      res     1
t4                      res     1
mask                    res       1 
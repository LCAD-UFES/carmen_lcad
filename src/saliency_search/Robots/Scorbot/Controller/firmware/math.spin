CON

        _clkmode        = xtal1 + pll16x
        _xinfreq        = 5_000_000


OBJ
  serial : "FullDuplexSerial"
  

VAR
        long  val1, val2, results
        
PUB main | dataIn
      serial.start(31, 30, 0,38400)
         

       val1 := -11023
       val2 := 50

      ' results := val1 + val2
       cognew(@mult, @val1) 
       dataIn := serial.rx
waitcnt(500_000 + cnt)

       serial.dec(val1)
       serial.str(string("/"))      
       serial.dec(val2)
       serial.str(string("="))
'       serial.bin(results, 32)
       serial.dec(results)              
       serial.str(string(10, 13))  

      



DAT

mult

        mov     addr, par
        add     addr, #0
        rdlong  _val1, addr

        mov     addr, par
        add     addr, #4
        rdlong  _val2, addr

        mov     addr, par
        add     addr, #8
        mov     _ptrResults, addr

        mov     mask, #$FF    'build mask
        shl     mask, #8
        add     mask, #$FF
        
        mov    t1, _val1
        mov    t2, _val2
        call    #divide
  
        
        
        wrlong  t1, _ptrResults

        jmp   #mult        


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


_val1         res       1
_val2         res       1
_ptrResults   res       1
t1            res       1
t2           res       1
t3           res       1
t4           res       1
addr          res       1
REsultMul     res       1
Multiplicant   res       1
Multiplier    res       1
mask          res       1
'long    $FFFF                       


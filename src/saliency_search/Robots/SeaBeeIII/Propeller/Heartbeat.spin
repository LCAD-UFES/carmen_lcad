VAR
   long stack[10]
   long heartcog
PUB Start(led)
  heartcog := cognew(Run(led), @stack)  
 
PUB Run(led)
  dira[led] := 1
  outa[led] := 0
  repeat
    !outa[led]
    waitcnt(clkfreq / 4 + cnt)


PUB Stop
cogstop(heartcog)      
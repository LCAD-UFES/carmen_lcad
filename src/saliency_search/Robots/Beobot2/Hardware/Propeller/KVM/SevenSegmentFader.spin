{{ Output.spin }}
CON
  DISP_DA = 8
  DISP_DB = 9
  DISP_DC = 10
  DISP_DD = 11
  DISP_DP = 12
  
VAR
  long  Stack[9]             'Stack space for new cog
  byte  val

PUB Start(Pin, Delay, Count)
{{Start new toggling process in a new cog.}}

  cognew(Toggle, @Stack)

PUB SetVal(v)
  val := v
PUB Toggle | duty, dir
  dira[DISP_DA .. DISP_DP]~~

  duty := 50
  dir := 1
  repeat
    if duty > 100
      dir := -1
    if duty < 0
      dir := 1
      
    duty := duty + dir
                  
    outa[DISP_DP .. DISP_DA] := val
    waitcnt(clkfreq/5000 + clkfreq/10000 * duty       + cnt)     '  Wait for Delay cycles
    outa[DISP_DP .. DISP_DA] := 15
    waitcnt(clkfreq/5000 + clkfreq/10000 * (100-duty) + cnt)     '  Wait for Delay cycles
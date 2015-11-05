CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

  LCD_PIN = 16
  DRAW = 1
OBJ
        lcd:"GraphicLcd"

PUB Main
  lcd.start(LCD_PIN)
	lcd.clearScreen
	lcd.clearScreen
	lcd.clearScreen
  lcd.vbar(0,0,20,100,78,1)
'	lcd.DrawSolidBox(0,0,100,50,1)
{ 
                lcd.movexy(70,0)
                lcd.str(string("CH1:"))
                lcd.dec(1000)
                lcd.movexy(14,0)
                lcd.str(string("CH2:"))
                lcd.dec(2000) 
}
  'lcd.grid(1)
  'lcd.ClearScreen      
  
              

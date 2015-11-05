CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

DAT
  'remote_pins        LONG 6, 5, 4, 3, 2, 1, 0
  'remote_pulseWidths LONG 1, 1, 1, 1, 1, 1, 1
OBJ
  serial                 : "ComputerLink"
        GPS : "GPS"

PUB Main | cmd, timeout

        timeout := CLKFREQ/4

  serial.start(31,30,0,115200)

        GPS.Start(13,12)
        
  repeat
                cmd := serial.rx
                        
                if(cmd == 0)
                        serial.frameStr(cmd, string("sensorboard"))

                elseif(cmd == 100) '<------------------------------- TODO: change the cmd number
                        'Get GPS time
                        serial.str (GPS.GetTime) 'format : "dd:mm:ss.mmm"

                elseif(cmd == 101) 
                        'Get Latitude                        
                        serial.str (GPS.GetLatitude) 'format : "dddÅãmm.mmmm' <N/S>" 
                        
                elseif(cmd == 103)
                        'Get Latitude                        
                        serial.str (GPS.GetLongitude)'format : "dddÅãmm.mmmm' <E/W>"

                elseif(cmd == 104)
                        'Get the satellite number
                        serial.str (GPS.GetSatelliteNumber) 'use this if you want fixed data dd (2digit)                       
                        'serial.dec (GPS.GetSatelliteNumberD)'else use this if you want just a 1digit if < 10, invalid data gives 255 (-1)

                elseif(cmd == 105)
                        'Get Precision Status
                        serial.str (GPS.GetPrecision) 'This only gives text, not actual data, like 'good', 'poor'
                        'serial.dec (GPS.GetPrecisionDD) 'these gives the actual value, invalid data gives 255 (-1)
                        'serial.dec (GPS.GetPrecisionM)  'format: dd.m                        

                elseif(cmd == 106)
                        'Get current speed 
                        serial.dec (GPS.GetSpeedDD) 'these gives the actual value, invalid data gives -1 (long)
                        serial.dec (GPS.GetSpeedMM) ' format: dd.mm        << the speed unit is knots 

                elseif(cmd == 107)
                        'Get current direction
                        serial.dec(GPS.GetDirectionDD) 'these gives the actual value, invalid data gives -1 (long)
                        serial.dec(GPS.GetDirectionMM) 'format: ddd.mm     << in degree, 0 is the north

                elseif(cmd == 97)
												'Send Lat/Lon 
												serial.tx(cmd)
												serial.tx(10)

												serial.tx(GPS.GetLatitudeDD)	
												serial.tx(GPS.GetLatitudeMM)	
												serial.sendShort(GPS.GetLatitudeMMMM)	

												serial.tx(GPS.GetLongitudeDD)	
												serial.tx(GPS.GetLongitudeMM)	
												serial.sendShort(GPS.GetLongitudeMMMM)	
												
												serial.tx(GPS.GetPrecisionDD)	

												serial.tx(GPS.GetSatelliteNumberD)	
														
												
												serial.tx(255)
																			

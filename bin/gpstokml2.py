#!/usr/bin/python

#coloca o timestamp
import string
import sys

nome_arqinput = sys.argv[1]
nome_arqoutput = nome_arqinput + """.kml""" 
arqinput = open(nome_arqinput, 'r')
arqoutput = open(nome_arqoutput, 'w')

latitude = 0
longitude = 0
altitude = 0
timestamp = 0
hora = 0
minuto = 0
segundo = 0
tempo = 0

output = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://earth.google.com/kml/2.2">
<!-- 

TimeStamp is recommended for Point.

Each Point represents a sample from a GPS.

-->

  <Document>
    <name>Points with TimeStamps</name>
    <Style id="hiker-icon">
      <IconStyle>
        <Icon>
          <href>http://www.google.com/mapfiles/ms/micons/cabs.png</href>
        </Icon>
        <hotSpot x="0" y=".5" xunits="fraction" yunits="fraction"/>
      </IconStyle>
    </Style>
    <Style id="check-hide-children">
      <ListStyle>
        <listItemType>checkHideChildren</listItemType>
      </ListStyle>
    </Style>
    <styleUrl>#check-hide-children</styleUrl>
"""

for line in arqinput:
       datablock = line.split(' ')

       if line[0:7] == 'NMEAGGA':
	       latitude_in = string.atof(datablock[3])
               longitude_in = string.atof(datablock[5])
               altitude = string.atof(datablock[11])
	       timestamp = string.atof(datablock[2])
	       hora = int(timestamp/10000)
	       minuto = int((timestamp/100)%100)
	       segundo = int(timestamp%100)
	       tempo = '''2011-06-01T''' + str(hora) + ''':'''+ str(minuto) + ''':'''+ str(segundo)

	       print str(hora)
	       print str(minuto)
	       print str(segundo)
	       
               if datablock[4] == 'S':
                        latitude_in = -latitude_in
               if datablock[6] == 'W':
                        longitude_in = -longitude_in

               latitude_degrees = int(latitude_in/100)
               latitude_minutes = latitude_in - latitude_degrees*100
  
               longitude_degrees = int(longitude_in/100)
               longitude_minutes = longitude_in - longitude_degrees*100

               latitude = latitude_degrees + (latitude_minutes/60)
               longitude = longitude_degrees + (longitude_minutes/60)
       
               if string.atof(datablock[7]) == 1:
		 outputlocal = ''
		 outputlocal = '''<Placemark>
      <TimeStamp>
        <when>'''+ tempo + '''</when>
      </TimeStamp>
      <styleUrl>#hiker-icon</styleUrl>
      <Point>
        <coordinates>''' + str(longitude) + ',' + str(latitude) + ',' + str(altitude) + '''</coordinates>
      </Point>
    </Placemark>'''
		 output = output + outputlocal   


output = output + """  </Document>
</kml>"""

arqoutput.write(output)
arqoutput.close()
arqinput.close()
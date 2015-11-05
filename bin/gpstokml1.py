#!/usr/bin/python
# Pinta o trajeto todo em vermelho
import string
import sys

nome_arqinput = sys.argv[1]
nome_arqoutput = nome_arqinput + """.kml""" 
arqinput = open(nome_arqinput, 'r')
arqoutput = open(nome_arqoutput, 'w')

latitude = 0
longitude = 0
altitude = 0


output = """<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://earth.google.com/kml/2.1">
<Style id="POI_STYLE">
    <IconStyle>
    <color>ff00ff00</color>
    <scale>1.1</scale>
    <Icon><href>http://maps.google.com/mapfiles/kml/pal3/icon21.png</href></Icon>
    </IconStyle>
</Style>
<Folder id="Data logger">
    <name>Data logger</name>
<Placemark id="logger">
  <styleUrl>#lineStyle</styleUrl>
   <description>Plot Your Traveling Path</description>
  <name>Trajectory</name>
    <visibility>1</visibility>
  <open>0</open>
    <Style>
    <LineStyle>
      <color>ff0000ff</color>
    </LineStyle>
    </Style>
    <LineString>
    <extrude>1</extrude>
    <tessellate>1</tessellate>
    <coordinates>
"""

for line in arqinput:
       datablock = line.split(' ')

       if line[0:7] == 'NMEAGGA':
	       latitude_in = string.atof(datablock[3])
               longitude_in = string.atof(datablock[5])
               altitude = string.atof(datablock[11])

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
		 output = output + str(longitude) + ',' + str(latitude) + ',' + str(altitude) + '\n'
output = output + """    </coordinates>
  </LineString>
</Placemark>
</Folder>
</kml>"""

arqoutput.write(output)
arqoutput.close()
arqinput.close()
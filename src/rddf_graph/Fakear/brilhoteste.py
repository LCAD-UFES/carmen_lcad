#!bin/python

import Image

imag = Image.open("testando2.png")
#Convert the image te RGB if it is a .gif for example
#imag = imag.convert ('RGB')
#coordinates of the pixel
#X,Y = 8,7
#Get RGB
#pixelRGB = imag.getpixel((X,Y))
pixelRGB = imag.getpixel((6,6))
R,G,B = pixelRGB 
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB

pixelRGB = imag.getpixel((6,7))
R,G,B = pixelRGB
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB

pixelRGB = imag.getpixel((6,8))
R,G,B = pixelRGB
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB

pixelRGB = imag.getpixel((7,6))
R,G,B = pixelRGB
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB

pixelRGB = imag.getpixel((7,7))
R,G,B = pixelRGB
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB

pixelRGB = imag.getpixel((7,8))
R,G,B = pixelRGB
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB

pixelRGB = imag.getpixel((8,6))
R,G,B = pixelRGB
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB

pixelRGB = imag.getpixel((8,7))
R,G,B = pixelRGB
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB

pixelRGB = imag.getpixel((8,8))
R,G,B = pixelRGB
brightness = sum([R,G,B])/3
#print brightness
print pixelRGB


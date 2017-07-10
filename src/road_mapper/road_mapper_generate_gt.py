from xml.dom import minidom
import numpy as np
import cv2
import struct

# Global definitions
BEZIER_INCREMENT = 0.001    # Line length increment (from 0.000 to 1.000) to set cubic Bezier curve points  
STROKE_INCREMENT = 0.05     # Line width increment  (in subpixels) to set the distance from the pixel to the line center 
MAX_INTENSITY = 255.0       # Maximum color intensity of pixel on the line center 
MIN_INTENSITY = 25.0        # Minimum color intensity of pixel on the line border
SHIFT = 16                  # Number of bits used for the fractionary part of a pixel coordinate 
MM_PER_PIXEL = 200.0        # Pixel length in millimeters

class road:
    def __init__(self):
        self.distance_center = 0    # short int: 'h'
        self.x_orientation = 0      # short int: 'h'
        self.y_orientation = 0      # short int: 'h'
        self.left_marking = 0       # char: 'b'
        self.right_marking = 0      # char: 'b'
         
      
#https://developer.mozilla.org/en-US/docs/Web/SVG/Tutorial/Paths
def svg_d_get_bezier_points(d):
    letter = ''
    count = 0
    points = [] # list of (x,y)
    last_abs_point_i = 0
    errors = 0
    n_ms = 0
    for p in d.split(' '):    
        if len(p) == 1:
            if p == 'm' or p == 'M' or p == 'c' or p == 'C' or p == 'l' or p == 'L' or p == 'h' or p == 'H' or p == 'v' or p == 'V':
                if p != letter:
                    count = 0
                letter = p
            else:
                errors += 1
        else:  
            if letter == 'm' or letter == 'M': # Move cursor to (x,y), lowercase = relative coordinates, uppercase = absolute coordinates
                pt = p.split(',')
                if n_ms > 0: # In case we have more than 1 (m or M)
                    points.append(points[len(points) - 1]) # Repeat last point just to fake the Bezier algorithm
                    if letter == 'm':
                        # calculate absolute point
                        delta = pt
                        points.append((float(delta[0]) + points[len(points) - 1][0], float(delta[1]) + points[len(points) - 1][1]))
                    else:
                        points.append((float(pt[0]), float(pt[1])))
                    points.append(points[len(points) - 1]) # Repeat new point just to fake the Bezier algorithm
                    last_abs_point_i = len(points) - 1
                else:
                    if len(pt) == 2:
                        points.append((float(pt[0]), float(pt[1])))
                        n_ms += 1
                    else:
                        errors += 1             
            elif letter == 'c': # Cubic Bezier curve, lowercase = relative coordinates
                count += 1   
                delta = p.split(',')
                # calculate absolute point
                if len(delta) == 2:                
                    pt[0] = float(delta[0]) + points[last_abs_point_i][0]
                    pt[1] = float(delta[1]) + points[last_abs_point_i][1]
                    points.append((float(pt[0]), float(pt[1])))     
                    if count % 3 == 0:
                        last_abs_point_i = len(points) - 1
                else:
                    errors +=1                            
            elif letter == 'C': # Cubic Bezier curve, uppercase = absolute coordinates
                count += 1            
                pt = p.split(',') 
                if len(pt) == 2:
                    points.append((float(pt[0]), float(pt[1])))
                    last_abs_point_i = len(points) - 1
                else:
                    errors +=1                            
            elif letter == 'l': # Draw straight line to next point (x,y), lowercase = relative coordinates
                delta = p.split(',')
                # calculate absolute point
                if len(delta) == 2:                
                    points.append(points[len(points) - 1]) # Repeat last point just to fake the Bezier algorithm
                    pt[0] = float(delta[0]) + points[last_abs_point_i][0]
                    pt[1] = float(delta[1]) + points[last_abs_point_i][1]
                    points.append((float(pt[0]), float(pt[1])))
                    points.append(points[len(points) - 1]) # Repeat new point just to fake the Bezier algorithm
                    last_abs_point_i = len(points) - 1
                else:
                    errors += 1             
            elif letter == 'L': # Draw straight line to next point (x,y), uppercase = absolute coordinates
                pt = p.split(',') 
                if len(pt) == 2:                
                    points.append(points[len(points) - 1]) # Repeat last point just to fake the Bezier algorithm
                    points.append((float(pt[0]), float(pt[1])))
                    points.append(points[len(points) - 1]) # Repeat new point just to fake the Bezier algorithm
                    last_abs_point_i = len(points) - 1
                else:
                    errors += 1             
            else:
                errors += 1
        if errors == 1:
            print 'Error in SVG line: d="', d, '"'
        if errors >= 1:
            print 'Unexpected SVG token: ', p
    return points

#https://stackoverflow.com/questions/15857818/python-svg-parser
def svg_get_paths(svg_file):
    doc = minidom.parse(svg_file)  # parseString also exists    
    paths = []
    img = doc.getElementsByTagName('image')
    width = int(img[0].getAttribute('width'))
    height = int(img[0].getAttribute('height'))
    for path in doc.getElementsByTagName('path'):
        d = path.getAttribute('d')
        points = svg_d_get_bezier_points(d)
        for style in path.getAttribute('style').split(';'):
            s = style.split(':')
            if s[0] == 'stroke-width':
                stroke_width = float(s[1]) 
            if s[0] == 'stroke': # this filed has the stroke color
                # stroke-color codes the lane marking according to readme.txt
                stroke_color = s[1]
        paths.append((points, stroke_width, stroke_color))
    doc.unlink()
    return width, height, paths

#https://stackoverflow.com/questions/785097/how-do-i-implement-a-b%C3%A9zier-curve-in-c
#https://stackoverflow.com/questions/37642168/how-to-convert-quadratic-bezier-curve-code-into-cubic-bezier-curve
def getPt(n1, n2 , perc):
    diff = n2 - n1;
    return n1 + ( diff * perc )
 
def get_bezier(width, height, points):  
    bx = []
    by = []
    btan = []
    for i in range((len(points)-1)/3):
        j = 0.0
        while j <= 1.0:
            # The Green Lines
            xa = getPt(points[i*3][0], points[i*3+1][0], j)
            ya = getPt(points[i*3][1], points[i*3+1][1], j)
            xb = getPt(points[i*3+1][0], points[i*3+2][0], j)
            yb = getPt(points[i*3+1][1], points[i*3+2][1], j)
            xc = getPt(points[i*3+2][0], points[i*3+3][0], j)
            yc = getPt(points[i*3+2][1], points[i*3+3][1], j) 
            # The Blue Line
            xm = getPt(xa, xb, j)
            ym = getPt(ya, yb, j)
            xn = getPt(xb, xc, j)
            yn = getPt(yb, yc, j)    
            # The Black Dot
            x = getPt(xm, xn, j)
            y = getPt(ym, yn, j)      
            bx.append(x)
            by.append(y)        
            if len(bx) > 1:
                x1 = x
                y1 = height - y # Y axis origin is upside down
                x0 = bx[-2]
                y0 = height - by[-2] # Y axis origin is upside down
                t = np.arctan2(y1 - y0 , x1 - x0)
                btan.append(t)
            j += BEZIER_INCREMENT
    btan2 = []
    btan2.append(btan[0])
    for i in range(1, len(btan)): # len(btan) = len(btan2)-1
        btan2.append((btan[i-1] + btan[i]) / 2.0)
    btan2.append(btan[len(btan)-1]) 
    return bx, by, btan2
#     #print len(bx), len(btan)
#     return bx, by, btan

def get_lane_from_bezier(img, bx, by, btan, stroke_width, stroke_color):
    MAX_R = int(stroke_color[1:3], 16)
    MAX_G = int(stroke_color[3:5], 16)
    MAX_B = int(stroke_color[5:8], 16)
#     for i in range(1, len(by)):
#         if i == 1:
#             atan0 = btan[0]
#             atan1 = (btan[0] + btan[1]) / 2
#         elif i == (len(by)-1):
#             atan0 = (btan[i-2] + btan[i-1]) / 2
#             atan1 = btan[i-1]
#         else:
#             atan0 = (btan[i-2] + btan[i-1]) / 2
#             atan1 = (btan[i-1] + btan[i]) / 2

    for i in range(1, len(bx)):
        atan0 = btan[i-1]
        atan1 = btan[i]
    
        vert = np.zeros((4,2),np.int32)
        vert[0, 0] = round(bx[i-1] * 2**SHIFT)
        vert[0, 1] = round(by[i-1] * 2**SHIFT)
        vert[3, 0] = round(bx[i]   * 2**SHIFT)
        vert[3, 1] = round(by[i]   * 2**SHIFT)
        
        k = stroke_width / 2
        while k >= 0.0:
            v = MAX_INTENSITY - (MAX_INTENSITY - MIN_INTENSITY) * k / (stroke_width / 2)
            r = MAX_R - (MAX_R - MIN_INTENSITY) * k / (stroke_width / 2)
            g = MAX_G - (MAX_G - MIN_INTENSITY) * k / (stroke_width / 2)
            b = MAX_B - (MAX_B - MIN_INTENSITY) * k / (stroke_width / 2)
            mm = round(k * MM_PER_PIXEL)
            vert[1, 0] = round((bx[i-1] + (k * np.sin(atan0))) * 2**SHIFT)
            vert[1, 1] = round((by[i-1] + (k * np.cos(atan0))) * 2**SHIFT)
            vert[2, 0] = round((bx[i]   + (k * np.sin(atan1))) * 2**SHIFT)
            vert[2, 1] = round((by[i]   + (k * np.cos(atan1))) * 2**SHIFT)
            #cv2.fillConvexPoly(img, vert, color=(b, g, r), lineType=8, shift=SHIFT)      
            cv2.fillConvexPoly(img, vert, color=(v, 0, 0), lineType=8, shift=SHIFT)
            vert[1, 0] = round((bx[i-1] - (k * np.sin(atan0))) * 2**SHIFT)
            vert[1, 1] = round((by[i-1] - (k * np.cos(atan0))) * 2**SHIFT)
            vert[2, 0] = round((bx[i]   - (k * np.sin(atan1))) * 2**SHIFT)
            vert[2, 1] = round((by[i]   - (k * np.cos(atan1))) * 2**SHIFT)
            #cv2.fillConvexPoly(img, vert, color=(b, g, r), lineType=8, shift=SHIFT) 
            cv2.fillConvexPoly(img, vert, color=(v, 0, 0), lineType=8, shift=SHIFT)
            k -= STROKE_INCREMENT
    return img

def get_lane_marking_by_color_code(stroke_color):
    if stroke_color == '#ff0000':
        l_marking = 'single_line'
        r_marking = 'single_line' 
    elif stroke_color == '#ff007f':
        l_marking = 'broken_line'
        r_marking = 'single_line' 
    elif stroke_color == '#7f00ff':
        l_marking = 'single_line'
        r_marking = 'broken_line' 
    elif stroke_color == '#0000ff':
        l_marking = 'broken_line'
        r_marking = 'broken_line' 
    return l_marking, r_marking

if __name__ == "__main__":
    svg_file =  'i7705600_-338380.00.svg'
    print 'Processing SVG file: ', svg_file
    cv2.namedWindow("original")
    cv2.namedWindow("median_blur")
    width, height, paths = svg_get_paths(svg_file)
    img = np.zeros((height, width, 3), np.uint8)
    for path in paths:
        bx, by, btan = get_bezier(width, height, path[0])
        img = get_lane_from_bezier(img, bx, by, btan, path[1], path[2])
        print get_lane_marking_by_color_code(path[2])
        img2 = cv2.medianBlur(img, 5)
    cv2.imshow("original", img)
    cv2.imshow("median_blur", img2)
    cv2.waitKey(0)
#     road_file = open('r' + svg_file[1:-3] + 'map', 'wb')
#     for i in range(height):
#         for j in range(width):
#             distance_center = int(MM_PER_PIXEL * (MAX_INTENSITY - img[i, j, 0]) * (path[1] / 2.0) / (MAX_INTENSITY - MIN_INTENSITY))
#             map = struct.pack('hhhbb', distance_center, 0,0,0,0)
#             road_file.write(map)
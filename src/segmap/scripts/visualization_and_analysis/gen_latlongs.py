
import os
import numpy as np

def degmin_to_double(dm):
    degree = np.floor(dm / 100.0)
    minutes = (dm - degree * 100.0) / 60.0
    return degree + minutes;

d = '/media/filipe/Hitachi-Teste' 
#d = '/dados'

logs = [l for l in os.listdir(d) if "log" in l and l[-4:] == '.txt']
for l in logs:
    print('Processing log', l)
    f = open(d + '/' + l, 'r')
    lines = [l.rstrip().rsplit() for l in f.readlines() if 'NMEAGGA' in l and '#' not in l] 
    g = open('gps_' + l, 'w')

    for l in lines:
        if l[1] != '1':
            continue
            
        lt = degmin_to_double(float(l[3]))
        lg = degmin_to_double(float(l[5]))
    
        if ('S' == l[4]): lt = -lt
        if ('W' == l[6]): lg = -lg
        
        g.write(str(lg) + ' ' + str(lt) + '\n')

    g.close()
    print('Done.')

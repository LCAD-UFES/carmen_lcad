
log = 'out.txt'

f = open(log, 'r')

s = f.readline()
lasth = 0
lastl = 0
line = 0

while s != '':
    splt_s = s.rsplit(' ')
    
    th = float(splt_s[len(splt_s) - 3].rsplit('.')[0])
    tl = float(splt_s[len(splt_s) - 3].rsplit('.')[1])

    if lasth > 0:
        if ((lasth > th) or ((lasth == th) and (lastl > tl))):
            print 'Problema na linha', line
    lasth = th
    lastl = tl
    s = f.readline()
    line += 1

f.close()


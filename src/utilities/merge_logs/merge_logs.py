
#3498
#s2 6 1487599130 392116 ** XSENS_QUAT -0.474776 0.366440 9.816052 0.395918 0.026758 -0.009897 -0.917843 -0.300479 0.149544 0.971958 -0.009042 0.002392 -0.004243 0.000000 0 1487599130.392116 car01 0.007302
#s1 6 1487599130 392116 ** XSENS_QUAT -0.474776 0.366440 9.816052 0.395918 0.026758 -0.009897 -0.917843 -0.300479 0.149544 0.971958 -0.009042 0.002392 -0.004243 0.000000 0 1487599130.392116 car01 0.007302
#s2 7 1487599130 392823 ** ROBOTVELOCITY_ACK 0.001000 -0.046468 1487599130.392823 car01 0.007376
#s2 8 1487599130 319918 ** BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3 /dados/log_ponte3-do_log_ponte-20170220.txt_bumblebee/1487590000/1487599100/1487599130.319918.bb3.image 1280 960 3686400 1 1487599130.319918 car01 0.196738
#s1 7 1487599130 392823 ** ROBOTVELOCITY_ACK 0.001000 -0.046468 1487599130.392823 car01 0.007376

log1 = '/dados/log_ponte3-do_log_ponte-20170220.txt'
log2 = '/dados/log_ponte3-do_log_ponte-20170220.txt'
logout = 'out.txt'

f1 = open(log1, 'r')
f2 = open(log2, 'r')
g = open(logout, 'w')

l1 = 0
l2 = 0
wl1 = 0
wl2 = 0
s1 = f1.readline()
s2 = f2.readline()

while (s1[0] == '#') or (s1[0:5] == 'PARAM'):
    l1 += 1
    s1 = f1.readline()

while (s2[0] == '#') or (s2[0:5] == 'PARAM'):
    l2 += 1
    s2 = f2.readline()

while (s1 != '') and (s2 != ''):
    splt_s1 = s1.rsplit(' ')
    splt_s2 = s2.rsplit(' ')

    th1 = int(splt_s1[len(splt_s1) - 3].rsplit('.')[0])
    th2 = int(splt_s2[len(splt_s2) - 3].rsplit('.')[0])

    tl1 = int(splt_s1[len(splt_s1) - 3].rsplit('.')[1])
    tl2 = int(splt_s2[len(splt_s2) - 3].rsplit('.')[1])

    if th1 < 1487500000:
        print 'Problema no log1 Linha ', l1
    if th2 < 1487500000:
        print 'Problema no log2 Linha ', l2

    if (th1 < th2) or ((th1 == th2) and (tl1 < tl2)):
        l1 += 1
        wl1 += 1
        g.write(s1)
        print 's1', wl1, th1, tl1, '**', s1
        raw_input()
        s1 = f1.readline()
    else:
        l2 += 1
        wl2 += 1
        print 's2', wl2, th2, tl2, '**', s2
        raw_input()
        g.write(s2)
        s2 = f2.readline()

while s1 != '':
    s1 = f1.readline()
    g.write(s1)

while s2 != '':
    s2 = f2.readline()
    g.write(s2)

f1.close()
f2.close()



f = open('../data/params.ini', 'r')
all_params = f.readlines()
f.close()

f = open('params_from_carmen.txt', 'r')
params_of_interest = [list(filter(None, l.rstrip().rsplit())) for l in f.readlines()]
f.close()

for l in params_of_interest:
    if len(l) <= 0:
        continue
    name = l[0]
    var = l[2]
    for p in all_params:
        if name in p:
            p = list(filter(None, p.rstrip().rsplit()))
            if p[1] == 'off': p[1] = 'false'
            elif p[1] == 'on': p[1] = 'true'

            print(var + ' = ' + p[1] + ';')
            break
        





import sys

filename = sys.argv[1]
f = open(filename, "r")
lines = f.readlines()
f.close()

g = open(filename, "w")

idx = 1
prev = []

for l in lines:
    s = l.rstrip().rsplit()
    if len(prev) > 0:
        if int(s[1]) < int(prev[1]):
            idx += 1
    s[2] = str(idx)
    g.write(' '.join(s) + '\n')
    prev = s

g.close()



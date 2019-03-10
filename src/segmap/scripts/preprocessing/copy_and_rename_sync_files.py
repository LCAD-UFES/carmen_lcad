
import os

d = '/dados/data/'
dirs = [f for f in os.listdir(d) if f[:4] == 'data']
for f in dirs:
    os.system('cp ' + d + '/' + f + '/sync.txt /home/filipe/sync_' + f)

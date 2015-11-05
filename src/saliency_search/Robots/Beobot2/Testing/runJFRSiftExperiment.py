#!/usr/bin/python

import os
import pexpect
import threading
from threading import Thread
import time

######################################################################
######################################################################
# Global Junk
######################################################################
#The original directory containing all of our video files on iRock
ROOT = '/lab/tmpir/u/JFRBeobotData2010/logs/'

#The directory where the sift.db, video.mgzJ, and logfiles will be
dest_dir = 'data/JFR10'

#A list of all nodes
nodes = ['bx1','bx2','bx4','bx5','bx6','bx7','bx8','bx3'] 

#The name of the master node
master = 'bx3'

#Which video directory to process
src_directory = '2010_06_05__15_08_31__20000-22999'

#A semaphore to keep the master from starting before the slaves
sem = None
######################################################################
######################################################################

class runWorker(Thread):
  node = ''
  child = None 
  running = True

  def __init__(self, node):
    Thread.__init__(self)
    self.node = node
  def run(self):

    #Spawn the ssh session to start up the worker process on the remote node
    cmd =  'ssh '+self.node+'@'+self.node
    cmd += ' "'
    cmd += ' killall -9 app-TestSIFT_Worker;'
    cmd += ' sleep 1; '
    cmd += ' ~/saliency/bin/app-TestSIFT_Worker '
    cmd += ' --siftdb /home/'+node+'/'+dest_dir+'/sift.db '
    cmd += ' 2>&1 | tee worker.log '
    cmd += '"'
    print cmd
    self.child = pexpect.spawn(cmd)
    self.child.expect('.ssword:*')
    self.child.sendline('itRocks')
    
    print self.node + ' ::: Opening Connection'
    self.child.expect('Connection opened')
    print self.node + ' ::: Opened'

    #Wait for the child to finish initializing
    print self.node + ': Loading DB and building KD Tree'

    self.child.expect('Waiting for server', timeout=None)
    sem.release()
    print self.node + ': Running!'

    #Just hang out until we get a kill signal from the main thread
    while(self.running == True):
      time.sleep(1)

    #Once we've gotten a kill signal, send a control-c to the worker
    self.child.sendcontrol('c')
    print self.node+': Finished...'

  def kill(self):
    print 'Killing '+self.node
    self.running = False

    #Murder the worker, just to be sure
    cmd = 'ssh '+self.node+'@'+self.node+' "killall -9 app-TestSIFT_Worker"'
    self.child = pexpect.spawn(cmd)
    self.child.expect('.ssword:*')
    self.child.sendline('itRocks')
    self.child.close()


#Copy the sift.db file over to the nodes, and compile the latest worker code
#for node in nodes:
#  #Make the JFR src_directory if it doesn't exist,
#  #and build the latest worker code
#  print node + ' ::: Building from latest source'
#  cmd =  'ssh '+node+'@'+node
#  cmd += ' "mkdir -p '+dest_dir+'; '
#  cmd += ' cd /home/'+node+'/saliency; '
#  cmd += ' svn up; '
#  cmd += ' make bin/app-TestSIFT_Worker" '
#  child = pexpect.spawn(cmd)
#  child.expect('password:*')
#  child.sendline('itRocks')
#  child.expect(pexpect.EOF, timeout=None)

#  #Copy the sift.db file over
#  print node + ' ::: Copying sift.db file'
#  cmd = 'scp '+ROOT+'/'+src_directory+'/sift.db '+node+'@'+node+':/home/'+node+'/'+dest_dir+'/'
#  child = pexpect.spawn(cmd, timeout=None)
#  child.expect('.ssword:*')
#  child.sendline('itRocks')
#  child.expect(pexpect.EOF, timeout=None)

##Copy the video over to the master
#print 'Copying Video File to Master: ' + master
#cmd = 'scp '+ROOT+'/'+src_directory+'/video.mgzJ '+master+'@'+master+':/home/'+master+'/'+dest_dir+'/'
#child = pexpect.spawn(cmd)
#child.expect('password:*')
#child.sendline('itRocks')
#child.expect(pexpect.EOF, timeout=None)

#Run experiment on increasing number of nodes
for numNodes in range(1, len(nodes)+1):
  print '------------------------------------------'
  threadList = []
  print 'Running on: ' + str(nodes[0:numNodes])

  #Initialize our semaphore with 0 resources, which will be added by our nodes
  #as they initialize
  sem = threading.Semaphore(0)

  for node in nodes[0:numNodes]:
    threadList.append(runWorker(node))
    threadList[-1].start() 

  #Acquire all of the release semaphore resources from the nodes to ensure that
  #they have all started up
  print master + ' ::: Sleeping'
  sem.acquire(len(threadList))
  print 'Starting Master: ' + master

  #Start up the master 
  cmd = 'ssh '+master+'@'+master+' '
  cmd += '"/home/'+master+'/saliency/bin/app-TestSIFT_Master '  #The application to run 
  cmd += '--in=/home/'+master+'/'+dest_dir+'/video.mgzJ '       #The input video file
  cmd += '/home/'+master+'/'+dest_dir+'/'+str(numNodes)+'.log ' #The log file name
  cmd += ' '.join(nodes[0:numNodes])                            #The list of nodes
  cmd += ' 2>&1 | tee master.log'
  cmd += '"'
  print master + ' ::: ' + cmd
  child = pexpect.spawn(cmd)
  child.expect('password:*')
  child.sendline('itRocks')


  print master + ' ::: Initializing Ice Runtime'
  child.expect('Initializing Ice Runtime')
  print master + ' ::: Initialized - waiting to finish'

  child.expect(pexpect.EOF, timeout=None)
  print 'Killing Master'
  child.sendcontrol('c')
  child.close()

  print 'Killing Nodes'

  #Kill all of the threads
  for thread in threadList:
    thread.kill()

  time.sleep(5)




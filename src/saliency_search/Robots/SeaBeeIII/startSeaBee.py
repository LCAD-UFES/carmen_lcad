#!/usr/bin/python

import os, subprocess, time, signal, sys
import atexit, errno, datetime

homePath = os.path.expanduser('~') + "/saliency/"
logPath = os.path.expanduser('~') + "/log/"
binPath = homePath + "bin/"

Programs = {}

    
def quit():
    for prog in Programs:
        Programs[prog]['popen'].kill() 
    icebox.kill()
    #os.system("rm -rf db/*")
    sys.exit()


startAll = sys.argv.count("--all")
    
if sys.argv.count("--gui") > 0:    
    # Start SeaBee3MainGUI
    Programs['SeaBee3MainGUI'] = \
        {
        'bin':'test-SeaBee3MainGUI',
        'arguments':[] \
        }

if sys.argv.count("--beestem") > 0 or startAll:    
    # Start BeeStemIce
    Programs['BeeStemIce'] = \
        {
        'bin':'test-BeeStemIce',
        'arguments':[] \
            }
            
if sys.argv.count("--particlefilter") > 0 or startAll:    
    # Start ParticleFilter
    Programs['ParticleFilter'] = \
        {
        'bin':'test-ParticleFilter',
        'arguments':['--out=none', \
        			 '--disable-graphics=1'] \
            }

if sys.argv.count("--LFwdRetina") > 0 or startAll:    
    # Start test-StereoVision w/ left fwd camera
    Programs['StereoRetina'] = \
        {
        'bin':'test-StereoRetina',
        'arguments':['--ice-identity=LFwdCamera'] \
            }

if sys.argv.count("--RFwdRetina") > 0 or startAll:    
    # Start test-StereoVision w/ right fwd camera
    Programs['StereoRetina'] = \
        {
        'bin':'test-StereoRetina',
        'arguments':['--ice-identity=RFwdCamera'] \
            }

if sys.argv.count("--imu") > 0 or startAll:    
    # Start IMUDataServer
    Programs['IMUDataServer'] = \
        {
        'bin':'app-IMUDataServer',
        'arguments':[] \
            }

if sys.argv.count("--fwd-retina") > 0 or startAll:    
    # Start Fwd Retina
    Programs['test-Retina'] = \
        {
        'bin':'test-Retina',
        'arguments':['--ice-identity=FwdCamera', '--in=V4L2', \
                         '--framegrabber-dev=/dev/video0',\
                         '--framegrabber-mode=YUYV', \
                         '--framegrabber-bswap=no'] \
    }

if sys.argv.count("--fwd-SV") > 0 or startAll:    
    # Start Fwd Retina
    #os.system("sudo chmod 777 /dev/bus/usb/001/005") #this seems to be the default; if it's set to a different port just change it manually
    Programs['test-SV'] = \
        {
        'bin':'test-SV',
        'arguments':['--out=none'] \
    }

if sys.argv.count("--dwn-retina") > 0 or startAll:    
    # Start Dwn Retina
    Programs['DwnRetina'] = \
        {
					'bin':'test-Retina',
        'arguments':['--ice-identity=DwnCamera', '--in=V4L2', \
                         '--framegrabber-dev=/dev/video1',\
                         '--framegrabber-mode=YUYV', \
                         '--framegrabber-bswap=no'] \
        }

'''
if sys.argv.count("--contour") > 0 or startAll:    
    # Start Vision Rectangle
    Programs['VisionRectangle'] = \
        {
					'bin':'test-VisionRectangle',
        'arguments':['--camera-source=all','--out=display'] \
            }
'''

if sys.argv.count("--hough") > 0 or startAll:    
    # Start Straight Line
    Programs['StraightEdgeFinder'] = \
        {
					'bin':'test-StraightEdgeFinder',
        'arguments':['--camera-source=DwnCamera','--out=none'] \
            }

# REMEMBER TO TAKE OUT MOTION CHANNEL !!
if sys.argv.count("--saliency") > 0 or startAll:    
    # Start Saliency Module
    Programs['SaliencyModule'] = \
        {
					'bin':'test-SaliencyModule',
        'arguments':['--camera-source=FwdCamera','--out=none'] \
        }


if sys.argv.count("--log") > 0 or startAll:

    date = datetime.datetime.now()
    dateStr = date.strftime("%Y_%m_%d_%H%M%S")
    datePath = logPath + dateStr
    
    os.system("mkdir " + datePath)
    os.system("mkdir " + datePath + "/fwd_img")
    os.system("mkdir " + datePath + "/dwn_img")
   
    # Start Logger
    Programs['LoggerModule'] = \
        {
					'bin':'test-LoggerModule',
        'arguments':['--log-path='+datePath] \
        }
    
# Check if icebox is already running
iceRunning = os.system("ps -ef | grep -v grep | grep icebox") 

if iceRunning == 0:
    os.system("killall -9 icebox")

# start IceBox
icebox = subprocess.Popen(['icebox','--Ice.Config=IceStorm.icebox', \
                               '--Ice.Nohup=1'])
time.sleep(1)
print "\n\nStarting icebox...\n\n"

for prog in Programs:
    cmd = Programs[prog]['arguments']
    cmd.insert(0,binPath+Programs[prog]['bin'])
#    cmd.insert(0,"nohup")
    time.sleep(1)
    print "\n\nStarting " + prog + "...\n\n"
    Programs[prog]['popen'] = subprocess.Popen(cmd)
    
    

# Check to make sure everything is running
# and if not, try and restart the process
signal.signal(signal.SIGINT,  lambda *args: quit())
signal.signal(signal.SIGQUIT, lambda *args: quit())

while sys.argv.count("--disable-auto-subs") <= 0:
    for prog in Programs:
        if Programs[prog]['popen'].poll() != None:
            cmd = Programs[prog]['arguments']
            
            Programs[prog]['popen'] = subprocess.Popen(cmd)
    time.sleep(2)

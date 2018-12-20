
import sys


GPS_TAG = 'NMEAGGA'
GPS_ORIENTATION_TAG = 'NMEAHDT'  
ODOM_TAG = 'ROBOTVELOCITY_ACK'
CAMERA_TAG = 'BUMBLEBEE_BASIC_STEREOIMAGE_IN_FILE3'
VELODYNE_TAG = 'VELODYNE_PARTIAL_SCAN_IN_FILE'
XSENS_TAG = 'XSENS_QUAT'  


def find_near(s, queue, key):
    near = 0
    for i in range(1, len(queue[key])):
        dti = abs(float(queue[key][i][-3]) - float(s[-3]))
        dtn = abs(float(queue[key][near][-3]) - float(s[-3]))
        if dti < dtn:
            near = i
    return near
    

def save_sync_data(queue, ref, order):
    keys_without_ref = [k for k in queue.keys() if k != ref]

    for s in queue[ref]:
        data = dict()
        data[ref] = s[:-2]
        
        for k in keys_without_ref:
            data[k] = queue[k][find_near(s, queue, k)][:-2]

        for k in order:        
            print(' '.join(data[k]), end=' ')
        print()


def add_to_queue(queue, data):
    queue.append(data)
    #if len(queue) > 100:
        #queue.pop(0)


def process_log(log, data):
    f = open(log, 'r')
    
    queue = {'gps1': [], 
        'gps_orientation': [], 
        'odom': [], 
        'camera': [], 
        'velodyne': [],
        'xsens': []}
    
    s = f.readline().rstrip().lstrip()
    while s != '':
        s = s.rsplit()
        s = list(filter(None, s))  # remove empty substrings due to more than one space in sequence.
        
        if len(s) > 2:
            if GPS_TAG in s[0] and s[1] == '1':
                add_to_queue(queue['gps1'], s)
            elif GPS_ORIENTATION_TAG in s[0]:
                add_to_queue(queue['gps_orientation'], s)
            elif ODOM_TAG in s[0]:
                add_to_queue(queue['odom'], s)
            elif CAMERA_TAG in s[0]:
                add_to_queue(queue['camera'], s)
            elif VELODYNE_TAG in s[0]:
                #save_sync_data(s, queue)
                #velodynes.append(s)
                add_to_queue(queue['velodyne'], s)
            elif XSENS_TAG in s[0]:
                add_to_queue(queue['xsens'], s)
            
        s = f.readline().rstrip().lstrip()     
    
    f.close()
    save_sync_data(queue, 'camera', ['velodyne', 'camera', 'gps1', 'gps_orientation', 'odom', 'xsens'])
    

if __name__ == "__main__":
    args = sys.argv
    
    if len(args) < 2:
        print("\n\nUse python %s [log name]\n\n" % args[0])
    else:
        logs = args[1:]
        data = []
        
        for log in logs:
            process_log(log, data)
        

import sys
import os
from glob import glob


def check_file_name(file_name):
    # file_name format: lane_<n>.txt
    try:
        n = int(file_name[5:-4])
        
        if file_name[:5] != 'lane_' or file_name[-4:] != '.txt':
            raise ValueError
        
    except ValueError:
        n = -1
    
    return n


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("\nUsage: python " + sys.argv[0] + " <labels_dir>\n")
        sys.exit(1)

    if not sys.argv[1].endswith('/'):
        sys.argv[1] += '/'
    
    if len(sys.argv[1].split('*')) > 2: 
        print("\nOnly one '*' allowed in pathname\n")
        sys.exit(1)

    label_file_list = glob(sys.argv[1] + '*')
    if not label_file_list:
        print("\nERROR: No label files found: " + sys.argv[1] + "\n")
        sys.exit(1)

    file_count = 0; file_error_name = 0; file_error_lines = 0; file_error_fields = 0; file_error_value = 0
    error_lines = [0] * 100

    for label_file in label_file_list:
        gt_file = os.path.basename(label_file)
        if not os.path.isfile(label_file):
            continue

        file_count += 1
        
        if check_file_name(gt_file) < 0:
            file_error_name += 1
            print('Error: file name: ' + label_file)
            continue
        
        file_lines = open(label_file).readlines()
        if len(file_lines) != 6:
            file_error_lines += 1
            if len(file_lines) < 100:
                error_lines[len(file_lines)] += 1
            print('Error: line count = %d ' % len(file_lines) + '  file = %s' % label_file)
            continue
        
        for line in file_lines:
            fields = line.split()
            if len(fields) != 5:
                file_error_fields += 1
                print('Error: field count = %d ' % len(fields) + '  line = %s' % line + '  file = %s' % label_file)
                continue
            
            for value in fields:
                try:
                    float(value)
                    
                except ValueError:
                    file_error_value += 1
                    print('Error: data value = %s' % value + '  line = %s' % line + '  file = %s' % label_file)
                        
    print('Total file count = %d' % file_count)
    print('Total file name errors = %d' % file_error_name)
    print('Total file line count errors = %d' % file_error_lines)
    for i in range(100):
        if error_lines[i] > 0:
            print('Total file line[%d] count errors = %d' % (i, error_lines[i]))
    print('Total file field count errors = %d' % file_error_fields)
    print('Total file data value errors = %d' % file_error_value)

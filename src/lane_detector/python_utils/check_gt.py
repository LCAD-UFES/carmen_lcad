import sys
import os


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
        print("\nUsage: python " + sys.argv[0] + " <ground_truth_dir>\n")
        sys.exit(1)

    gt_subdirs = [subdir for subdir in os.listdir(sys.argv[1])]
    
    file_count = 0; file_error_name = 0; file_error_lines = 0; file_error_fields = 0; file_error_value = 0
    error_lines = [0] * 100
    
    for subdir in gt_subdirs:
        subdir_name = sys.argv[1] + '/' + subdir + '/labels'
        if not os.path.isdir(subdir_name):
            continue

        gt_files = [files for files in os.listdir(subdir_name)]

        for gt_file in gt_files:
            file_name = subdir_name + '/' + gt_file
            if not os.path.isfile(file_name):
                continue

            file_count += 1
            
            if check_file_name(gt_file) < 0:
                file_error_name += 1
                print('Error: file name: ' + file_name)
                continue
            
            file_lines = open(file_name).readlines()
            if len(file_lines) != 6:
                file_error_lines += 1
                if len(file_lines) < 100:
                    error_lines[len(file_lines)] += 1
                print('Error: line count = %d ' % len(file_lines) + '  file = %s' % file_name)
                continue
            
            for line in file_lines:
                fields = line.split()
                if len(fields) != 5:
                    file_error_fields += 1
                    print('Error: field count = %d ' % len(fields) + '  line = %s' % line + '  file = %s' % file_name)
                    continue
                
                for value in fields:
                    try:
                        float(value)
                        
                    except ValueError:
                        file_error_value += 1
                        print('Error: data value = %s' % value + '  line = %s' % line + '  file = %s' % file_name)
                        
    print('Total file count = %d' % file_count)
    print('Total file name errors = %d' % file_error_name)
    print('Total file line count errors = %d' % file_error_lines)
    for i in range(100):
        if error_lines[i] > 0:
            print('Total file line[%d] count errors = %d' % (i, error_lines[i]))
    print('Total file field count errors = %d' % file_error_fields)
    print('Total file data value errors = %d' % file_error_value)

#time ./velodyne_calibration_save_data /dados/log_volta_da_ufes-20180112.txt ../../carmen-ford-escape.ini /tmp/calib-data.txt --use_calib 0 --ignore_above_threshold 100 --ignore_below_threshold -100
#awk '{print $0" "int($5 / 0.2)" "int($6 / 0.2)}' /tmp/calib-data.txt > /tmp/calib-data-2.txt
#sort -k8,8n -k9,9n /tmp/calib-data-2.txt > /tmp/calib-data-sorted.txt
#mv /tmp/calib-data-sorted.txt ./
python velodyne_calibration_compute_calibration_table.py

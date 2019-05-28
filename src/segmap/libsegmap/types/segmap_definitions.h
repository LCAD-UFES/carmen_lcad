
#ifndef _SEGMAP_CAR_CONFIG_H_
#define _SEGMAP_CAR_CONFIG_H_

// IARA
#define distance_between_rear_wheels 1.535
#define distance_between_front_and_rear_axles 2.625
#define distance_between_front_car_and_front_wheels 0.85
#define distance_between_rear_car_and_rear_wheels 0.96

// Ford fusion
//#define distance_between_rear_wheels 1.585
//#define distance_between_front_and_rear_axles 2.85
//#define distance_between_front_car_and_front_wheels 0.94
//#define distance_between_rear_car_and_rear_wheels 1.13

// All
#define car_length (distance_between_front_and_rear_axles + distance_between_rear_car_and_rear_wheels + distance_between_front_car_and_front_wheels)
#define car_width distance_between_rear_wheels
#define center_to_rear_axis (car_length / 2. - distance_between_rear_car_and_rear_wheels)
#define TIME_SPENT_IN_EACH_SCAN 0.000046091445
const int MAX_RANGE = 70;

#endif

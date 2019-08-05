
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

class CityscapesObjectClassMapper
{
public:
	static unsigned char transform_object_class(unsigned int object_class)
	{
		return object_class;

		/*
		if (object_class == 0) return 0;

		// CLASSES:
		// road 0
		// sidewalk 1
		// building 2
		// wall 3
		// fence 4
		// pole 5
		// traffic_light 6
		// traffic_sign 7
		// vegetation 8
		// terrain 9
		// sky 10
		// person 11
		// rider 12
		// car 13
		// truck 14
		// bus 15
		// train 16
		// motorcycle 17
		// bicycle 18
		// unknown 19
		// lane marks 20

		// Deeplabv3+ usually mistakes the classes sidewalk and terrain, so we map them to the same class.
		if (object_class == 1 || object_class == 8)
			return 8;

		// building, wall, fence -> building
		if (object_class == 2 || object_class == 3 || object_class == 4)
			return 2;

		// traffic_light, traffic_sign, pole -> traffic_sign
		if (object_class == 6 || object_class == 7 || object_class == 5)
			return 6;

		// person, rider, bicycle -> unknown (disconsidered for localization and mapping)
		if (object_class == 11 || object_class == 12)
			return 19;

		// we assume that these vehicles are only located in road-like places.
		// car, trucks, bus, train, motorcycle -> road
		if (object_class == 13 || object_class == 14 || object_class == 15 || object_class == 16 || object_class == 17)
			return 19;

		return object_class;
		*/
	}
};

#endif

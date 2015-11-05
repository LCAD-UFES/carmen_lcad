/*
 * landmark_map.h
 *
 *  Created on: Apr 12, 2013
 *      Author: filipe
 */

#ifndef _LANDMARK_MAP_H_
#define _LANDMARK_MAP_H_

#include <vector>
using namespace std;

namespace post_slam
{
	class LandmarkMapParams
	{
		public:
			static double max_post_radius;
	};

	class LandmarkMap
	{
		vector<Landmark*> landmarks;
		Landmark* find_nearest_landmark(Landmark* landmark);

		public:
			LandmarkMap();
			~LandmarkMap();

			void add(vector<Landmark*> *new_landmarks);
			void add(Landmark *landmark);

			Landmark* match(Landmark*);
			vector<Landmark*> match(vector<Landmark*> *landmarks);

			void save(char *filename);
			void load(char *filename);
	};
}

#endif /* _LANDMARK_MAP_H_ */

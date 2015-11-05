/*
 * street.h
 *
 *  Created on: 08/11/2012
 *      Author: romulo
 */

#ifndef STREET_H_
#define STREET_H_

using namespace std;
#include <carmen/carmen.h>


class Lane
{

public:
	static vector<carmen_point_t> get_street(const char* kml_path);
};

#endif /* STREET_H_ */

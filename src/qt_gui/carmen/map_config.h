/*
 * map_config.h
 *
 *  Created on: 09/12/2011
 *      Author: rradaelli
 */

#ifndef MAP_CONFIG_H_
#define MAP_CONFIG_H_

class Map_Config {
public:
	Map_Config();
	virtual ~Map_Config();

	int x_size;
	int y_size;
	double resolution;
	char *map_name;
};

#endif /* MAP_CONFIG_H_ */

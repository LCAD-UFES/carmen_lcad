/*
    simple line fitting
    Copyright (C) 2009 Bob Mottram and Giacomo Spigler
    fuzzgun@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef LINEFIT_H_
#define LINEFIT_H_

#include <stdio.h>

#define LINE_SLOPES           20
#define LINE_MAX_IMAGE_WIDTH  640
#define LINE_SAMPLING         1
#define MAX_LINES             10

class linefit {
public:
	unsigned int bucket[LINE_SLOPES*2+1][3*LINE_MAX_IMAGE_WIDTH/LINE_SAMPLING];
    int line_horizontal[1 + (MAX_LINES*5)];
    int line_vertical[1 + (MAX_LINES*5)];
    unsigned int best_lines[MAX_LINES*2];

    void parallel(
        int* lines,
        int tollerance);

	void vertically_oriented(
		int no_of_feats,
		short int* feature_x,
		unsigned short int* features_per_row,
		int vertical_sampling,
		int minimum_edges);

	void horizontally_oriented(
		int no_of_feats,
		short int* feature_y,
		unsigned short int* features_per_col,
		int horizontal_sampling,
		int minimum_edges);
};

#endif

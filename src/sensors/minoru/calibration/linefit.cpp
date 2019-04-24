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

#include "linefit.h"
#include <string.h>

void linefit::parallel(
    int* lines,
    int tollerance) {
	int s0,s1,ds, best_s=0,hits,i,j,n = lines[0];
	int max_hits = 0;
	for (i = 0; i < n; i++) {
		s0 = lines[i*5+5];
		hits = 0;
		for (j = 0; j < n; j++) {
			if (i != j) {
				s1 = lines[i*5+5];
				ds = s1 - s0;
				if ((ds > -tollerance) &&
				    (ds < tollerance)) {
				    hits++;
				}
			}
		}
		if (hits > max_hits) {
			max_hits = hits;
			best_s = s0;
		}
	}
	if (max_hits > 5) {
		for (i = 0; i < n; i++) {
			s0 = lines[i*5+5];
			ds = best_s - s0;
			if ((ds <= -tollerance) ||
				(ds >= tollerance)) {
				lines[i*5+1] = 0;
				lines[i*5+2] = 0;
				lines[i*5+3] = 0;
				lines[i*5+4] = 0;
			}
		}
	}
	else {
		lines[0] = 0;
	}
}

void linefit::vertically_oriented(
	int no_of_feats,
	short int* feature_x,
	unsigned short int* features_per_row,
	int vertical_sampling,
	int minimum_edges)
{
	int x,y,f,s,b,i,j,k,n, bucket_index,row;
	int offset = LINE_MAX_IMAGE_WIDTH/LINE_SAMPLING;
	int max_bucket = 3*LINE_MAX_IMAGE_WIDTH/LINE_SAMPLING;
	int feats_remaining, tx=0,ty=0,bx=0,by=0;
	unsigned int avg, hits, v;

	/* clear number of lines */
	line_vertical[0] = 0;

	/* clear the buckets */
	for (s = -LINE_SLOPES; s <= LINE_SLOPES; s++) {
	    memset((void*)bucket[s + LINE_SLOPES], '\0', max_bucket * sizeof(unsigned int));
	}

	/* populate buckets */
	row = 0;
	feats_remaining = features_per_row[row];
	for (f = 0; f < no_of_feats; f++, feats_remaining--) {

		x = (int)feature_x[f];
		y = 4 + (row * vertical_sampling);

		for (s = -LINE_SLOPES; s <= LINE_SLOPES; s++) {
            bucket_index = offset + ((x + (s * y / LINE_SLOPES)) / LINE_SAMPLING);
            if ((bucket_index >= 0) &&
            	(bucket_index < max_bucket)) {
                bucket[LINE_SLOPES + s][bucket_index]++;
            }
		}

		/* move to the next row */
		if (feats_remaining <= 0) {
			row++;
			feats_remaining = features_per_row[row];
		}
	}

	hits = 0;
	avg = 0;
	for (s = -LINE_SLOPES; s <= LINE_SLOPES; s++) {
	    for (b = 0; b < max_bucket; b++) {
		    if (bucket[s + LINE_SLOPES][b] > 0) {
		    	//if (bucket[s + LINE_SLOPES][b] > 1000) printf("bucket %d: %d\n", b, bucket[s + LINE_SLOPES][b]);
		    	hits++;
		    	avg += bucket[s + LINE_SLOPES][b];
		    }
	    }
	}
	if (hits > 0) {
		i = 0;
		memset((void*)best_lines, '\0', MAX_LINES*2*sizeof(unsigned int));
		avg /= hits;
		if ((int)avg < minimum_edges) avg = minimum_edges;
		n = 0;
		for (s = -LINE_SLOPES; s <= LINE_SLOPES; s++) {
		    for (b = 0; b < max_bucket; b++, n++) {
		    	v = bucket[s + LINE_SLOPES][b];
			    if (v > avg) {
			    	if (bucket[s + LINE_SLOPES][b] > 1000) printf("v: %d\n", v);
                    for (j = 0; j < MAX_LINES; j++) {
                        if (best_lines[j*2] < v) {
                            for (k = MAX_LINES-1; k > j; k--) {
                            	best_lines[k*2] = best_lines[(k-1)*2];
                            	best_lines[k*2 + 1] = best_lines[(k-1)*2 + 1];
                            }
                            best_lines[j*2] = v;
                            best_lines[j*2 + 1] = n;
                            i++;
                            break;
                        }
                    }
			    }
		    }
		}
		if (i > MAX_LINES) i = MAX_LINES;
		for (j = 0; j < i; j++) {
			n = best_lines[j*2+1];
			b = n % max_bucket;
			s = ((n - b) / max_bucket) - LINE_SLOPES;

			tx = -1;
			row = 0;
			feats_remaining = features_per_row[row];
			for (f = 0; f < no_of_feats; f++, feats_remaining--) {

				x = (int)feature_x[f];
				y = 4 + (row * vertical_sampling);

	            if (offset + ((x + (s * y / LINE_SLOPES)) / LINE_SAMPLING) == b) {
	            	if (tx == -1) {
            		    tx = x;
            		    ty = y;
	            	}
	            	bx = x;
	            	by = y;
	            }

				/* move to the next row */
				if (feats_remaining <= 0) {
					row++;
					feats_remaining = features_per_row[row];
				}
			}

			n = line_vertical[0];
			line_vertical[n*5+1] = tx;
			line_vertical[n*5+2] = ty;
			line_vertical[n*5+3] = bx;
			line_vertical[n*5+4] = by;
			line_vertical[n*5+5] = s;
			line_vertical[0]++;
			if (line_vertical[0] == MAX_LINES) break;
		}
	}
	parallel(line_vertical,LINE_SLOPES*15/100);
}

void linefit::horizontally_oriented(
	int no_of_feats,
	short int* feature_y,
	unsigned short int* features_per_col,
	int horizontal_sampling,
	int minimum_edges)
{
	int x,y,f,s,b,i,j,k,n, bucket_index,col;
	int offset = LINE_MAX_IMAGE_WIDTH/LINE_SAMPLING;
	int max_bucket = 3*LINE_MAX_IMAGE_WIDTH/LINE_SAMPLING;
	int feats_remaining, tx=0,ty=0,bx=0,by=0;
	unsigned int avg, hits, v;

	/* clear number of lines */
	line_horizontal[0] = 0;

	/* clear the buckets */
	for (s = -LINE_SLOPES; s <= LINE_SLOPES; s++) {
	    memset((void*)bucket[s + LINE_SLOPES], '\0', max_bucket * sizeof(unsigned int));
	}

	/* populate buckets */
	col = 0;
	feats_remaining = features_per_col[col];
	for (f = 0; f < no_of_feats; f++, feats_remaining--) {

		x = 4 + (col * horizontal_sampling);
		y = (int)feature_y[f];

		for (s = -LINE_SLOPES; s <= LINE_SLOPES; s++) {
            bucket_index = offset + ((y + (s * x / LINE_SLOPES)) / LINE_SAMPLING);
            if ((bucket_index >= 0) &&
            	(bucket_index < max_bucket)) {
                bucket[LINE_SLOPES + s][bucket_index]++;
            }
		}

		/* move to the next row */
		if (feats_remaining <= 0) {
			col++;
			feats_remaining = features_per_col[col];
		}
	}

	hits = 0;
	avg = 0;
	for (s = -LINE_SLOPES; s <= LINE_SLOPES; s++) {
	    for (b = 0; b < max_bucket; b++) {
		    if (bucket[s + LINE_SLOPES][b] > 0) {
		    	hits++;
		    	avg += bucket[s + LINE_SLOPES][b];
		    }
	    }
	}
	if (hits > 0) {
		i = 0;
		memset((void*)best_lines, '\0', MAX_LINES*2*sizeof(unsigned int));
		avg /= hits;
		if ((int)avg < minimum_edges) avg = minimum_edges;
		n = 0;
		for (s = -LINE_SLOPES; s <= LINE_SLOPES; s++) {
		    for (b = 0; b < max_bucket; b++, n++) {
		    	v = bucket[s + LINE_SLOPES][b];
			    if (v > avg) {
                    for (j = 0; j < MAX_LINES; j++) {
                        if (best_lines[j*2] < v) {
                            for (k = MAX_LINES-1; k > j; k--) {
                            	best_lines[k*2] = best_lines[(k-1)*2];
                            	best_lines[k*2 + 1] = best_lines[(k-1)*2 + 1];
                            }
                            best_lines[j*2] = v;
                            best_lines[j*2 + 1] = n;
                            i++;
                            break;
                        }
                    }
			    }
		    }
		}
		if (i > MAX_LINES) i = MAX_LINES;
		for (j = 0; j < i; j++) {
			n = best_lines[j*2+1];
			b = n % max_bucket;
			s = ((n - b) / max_bucket) - LINE_SLOPES;

			tx = -1;
			col = 0;
			feats_remaining = features_per_col[col];
			for (f = 0; f < no_of_feats; f++, feats_remaining--) {

				x = 4 + (col * horizontal_sampling);
				y = (int)feature_y[f];

	            if (offset + ((y + (s * x / LINE_SLOPES)) / LINE_SAMPLING) == b) {
	            	if (tx == -1) {
	            		tx = x;
	            		ty = y;
	            	}
	            	bx = x;
	            	by = y;
	            }

				/* move to the next row */
				if (feats_remaining <= 0) {
					col++;
					feats_remaining = features_per_col[col];
				}
			}

			n = line_horizontal[0];
			line_horizontal[n*5+1] = tx;
			line_horizontal[n*5+2] = ty;
			line_horizontal[n*5+3] = bx;
			line_horizontal[n*5+4] = by;
			line_horizontal[n*5+5] = s;
			line_horizontal[0]++;
			if (line_horizontal[0] == MAX_LINES) break;
		}
	}
	parallel(line_horizontal,LINE_SLOPES*15/100);
}

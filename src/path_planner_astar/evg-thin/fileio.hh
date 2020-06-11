/*********************************************************************

  EVG-THIN v1.1: A Thinning Approximation to the Extended Voronoi Graph
  
  Copyright (C) 2006 - Patrick Beeson  (pbeeson@cs.utexas.edu)


  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301
  USA

*********************************************************************/


#ifndef fileio_hh
#define fileio_hh 

#include "datatypes.hh"
#include "ImageHelper.hh"


/** 
    Uses ImageHelper to read in a pixmap then converts it into a
    vector of vectors.  Each cell is either Free, Occupied, or Unknown
    (basically a simple occupancy grid from the robotics literature).
**/
class fileio {
public:
  ~fileio() {
    delete input_image;
  }

  fileio() {
    input_image=NULL;
  }

  //Reads in a .ppm or .pgm and coverts it to a occupancy grid.
  grid_type read_file(const char* filename,
		     unsigned int unknown_min,
		     unsigned int unknown_max) {

    input_image = new ImageHelper(filename);


    input_image->greyscale();

    column_type col(input_image->height(),Unknown);
    grid_type newgrid(input_image->width(),col);
    
    for (int i=0; i<input_image->width();i++)
      for (int j=0; j<input_image->height();j++) {
	unsigned char cell= *(input_image->buffer(i,j));
	if (cell > unknown_max)
	  newgrid[i][j]=Free;
	else 
	  if (cell < unknown_min)
	    newgrid[i][j]=Occupied;
      }

    return newgrid;
  }
  

  //Saves a .ppm file with the original occupnacy grid along with a
  //red "skeleton of freespace".
  void save_file(const skeleton_type& skel, const char* filename) {
    unsigned char red[3]={255,0,0};

    if (input_image != NULL) {
      for (unsigned int i=0;i<skel.size();i++)
	input_image->put(skel[i].x,skel[i].y,red);
      
      input_image->save(filename);
    }
  }

private:
  ImageHelper* input_image;

};


#endif

#if !defined(geo_Gdc_Coord_3d)
#define geo_Gdc_Coord_3d

//
// Filename: Gdc_Coord_3d.h
//
// Author: Dan Toms, SRI International
//
// Package: GeoTransform <http://www.ai.sri.com/geotransform/>
//
// Acknowledgements:
//   The algorithms used in the package were created by Ralph Toms and
//   first appeared as part of the SEDRIS Coordinate Transformation API.
//   These were subsequently modified for this package. This package is
//   not part of the SEDRIS project, and the Java code written for this
//   package has not been certified or tested for correctness by NIMA. 
//
// License:
//   The contents of this file are subject to GeoTransform License Agreement
//   (the "License"); you may not use this file except in compliance with
//   the License. You may obtain a copy of the License at
//   http://www.ai.sri.com/geotransform/license.html
//
//   Software distributed under the License is distributed on an "AS IS"
//   basis, WITHOUT WARRANTY OF ANY KIND, either express or implied. See
//   the License for the specific language governing rights and limitations
//   under the License.
//
//   Portions are Copyright (c) SRI International, 1998.
//
//   This version translated into C++ by Malcolm Corbin, QinetiQ, April 2001
//

//package geotransform.coords;

/*
 * Class: Gdc_Coord_3d
 * 
 * Description: *//**
 *   Represents a geodetic (lat, long, elev) coordinate.
 *  
 *   This class is used to represent a single geodetic, or lat/long,
 *   coordinate, i.e. angular data relative to the center of the
 *   ellipsoid. This includes a latitude value (degrees), a longitude
 *   value (degrees), and an elevation value (meters).
 *
 * @author Dan Toms, SRI International
 *
 * @version $Id: Gdc_Coord_3d.java,v 1.1.1.1 1999/11/01 03:00:56 reddy Exp $
 *
 * @see Gcc_Coord_3d
 * @see Utm_Coord_3d
 * */

#include <string>

class Gdc_Coord_3d {

public: 
	double longitude;
	double latitude;
	double elevation;
  
  /*
   * Method: Gdc_Coord_3d
   * 
   * Description: *//**
   *   Constructor for Gdc_Coord_3d.
   *  
   *   This constructor will create a new instance of the Gdc_Coord_3d
   *   class. It accepts no parameters. All of the class's member
   *   variables will be set to their default initialization values (0).
   *
   * @since  1.0
   *
   */

  Gdc_Coord_3d();

  /*
   * Method: Gdc_Coord_3d
   * 
   * Description: *//**
   *   Constructor for Gdc_Coord_3d.
   *  
   *   This constructor will create a new instance of the Gdc_Coord_3d
   *   class. It lets you specify the default value for all of the 
   *   member variable of the class.
   *
   * @param lat  the latitude of the coordinate (degrees)
   * @param lat  the longitude of the coordinate (degrees)
   * @param e    the elevation of the coordinate (meters)
   *
   * @since  1.0
   *
   */

  Gdc_Coord_3d( double lat, double lon, double e );

  // convert to a string representation
  std::string toString();
    
};

#endif // !defined(geo_Gdc_Coord_3d)

#if !defined(geo_Gdc_To_Utm_Converter)
#define geo_Gdc_To_Utm_Converter

//
// Filename: Gdc_To_Utm_Converter.h
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



/*
 * Class: Gdc_To_Utm_Converter
 * 
 * Description: *//**
 *   Converts GDC coordinate(s) to UTM.
 *  
 *   This class provides the capability to convert from
 *   geodetic (GDC), i.e. lat/long coordinates to 
 *   Universal Transverse Mercator (UTM).
 *   Methods are provided to convert either a single point
 *   or an array of points. This is a direct conversion.
 *
 * @author Dan Toms, SRI International
 *
 * @version $Id: Gdc_To_Utm_Converter.java,v 1.1.1.1 1999/11/01 03:00:56 reddy Exp $
 *
 * @see Gcc_To_Gdc_Converter
 * @see Gcc_To_Utm_Converter
 * @see Utm_To_Gcc_Converter
 * @see Utm_To_Gdc_Converter
 * @see Gdc_To_Gcc_Converter
 *
 */

class Gdc_Coord_3d;
class Utm_Coord_3d;

class Gdc_To_Utm_Converter
{
private:
    static const double RADIANS_PER_DEGREE;
    static const double PI;
    
    static double A,
                  F,
                  C,
                  Eps2,
                  Eps25,
                  Epps2,   
                  CScale,
                  poly1b,
                  poly2b,
                  poly3b,
                  poly4b,
                  poly5b;

   /*
    * Method: Init
    * 
    * Description: *//**
    *   Initializes the class for a specific ellipsoid.
    *  
    *   This method will set up various internal variables
    *   that are used to perform the coordinate conversions.
    *   You need to supply an ellipsoid to initialise these
    *   conversions.
    *   <p>
    *   You MUST call one of the Init methods before calling any of 
    *   the Convert methods.
    *
    * @param a   the semi-major axis (meters) for the ellipsoid
    * @param f   the inverse flattening for the ellipsoid
    *
    * @return void
    *
    * @since  1.0
    *
    */

//    public static void Init(double a, double f)
    
   /*
    * Method: Init
    * 
    * Description: *//**
    *   Initializes the class for a specific ellipsoid.
    *  
    *   This method will set up various internal variables
    *   that are used to perform the coordinate conversions.
    *   The WGS 84 ellipsoid will be assumed.
    *   <p>
    *   You MUST call one of the Init methods before calling any of 
    *   the Convert methods.
    *
    * @return void
    *
    * @since  1.0
    *
    */

public: 
	static void Init();
    
   /*
    * Method: Init
    * 
    * Description: *//**
    *   Initializes the class for a specific ellipsoid.
    *  
    *   This method will set up various internal variables
    *   that are used to perform the coordinate conversions.
    *   You need to supply an ellipsoid to initialise these
    *   conversions.
    *   <p>
    *   You MUST call one of the Init methods before calling any of 
    *   the Convert methods.
    *
    * @param E   an Ellipsoid instance for the ellipsoid, e.g. WE_Ellipsoid
    *
    * @return void
    *
    * @since  1.0
    *
    * @see geotransform.ellipsoids.Ellipsoid
    *
    */

//    public static void Init(Ellipsoid E)
                   
protected: 
	static void CreateConstants(double a, double f);

   /*
    * Method: Convert
    * 
    * Description: *//**
    *   Performs a single coordinate transform.
    *  
    *   This function will transform a single coordinate. The
    *   input coordinate is provided in the Gdc_Coord_3d instance.
    *   The result of the transformation will be put in the
    *   Utm_Coord_3d instance.
    *
    * @param gdc_coord   the input GDC coordinate
    * @param utm_coord   the output UTM coordinate
    *
    * @return void
    *
    * @since  1.0
    *
    * @see geotransform.coords.Gdc_Coord_3d
    * @see geotransform.coords.Utm_Coord_3d
    *
    */

public: 
	static void Convert(Gdc_Coord_3d gdc_coord, Utm_Coord_3d& utm_coord);
    
   /*
    * Method: Convert
    * 
    * Description: *//**
    *   Performs multiple coordinate transforms.
    *  
    *   This function will transform any array
    *   of coordinates. The
    *   input coordinate array is provided in the Gdc_Coord_3d[] instance.
    *   The results of the transformation will be put into each element
    *   of the Utm_Coord_3d[] instance. You should have at least as
    *   many entries in the target array as exist in the source array.
    *
    * @param gdc   the input array of GDC coordinates
    * @param utm   the output array of UTM coordinates
    *
    * @return void
    *
    * @since  1.0
    *
    * @see geotransform.coords.Gdc_Coord_3d
    * @see geotransform.coords.Utm_Coord_3d
    *
    */
    
public: 
	static void Convert(int count, const Gdc_Coord_3d gdc[], Utm_Coord_3d utm[] );
    
}; // end 

#endif // !defined(geo_Gdc_To_Utm_Converter)

#if !defined(geo_Utm_To_Gdc_Converter)
#define geo_Utm_To_Gdc_Converter

//
// Filename: Utm_To_Gdc_Converter.h
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
 * Class: Utm_To_Gdc_Converter
 * 
 * Description: *//**
 *   Converts UTM coordinate(s) to GDC.
 *   <p>
 *   This class provides the capability to convert from
 *   Universal Transverse Mercator (UTM) coordinates to 
 *   geodetic (GDC), i.e. lat/long.
 *   Methods are provided to convert either a single point
 *   or an array of points. This is a direct conversion.
 *
 * @author Dan Toms, SRI International
 *
 * @version $Id: Utm_To_Gdc_Converter.java,v 1.1.1.1 1999/11/01 03:00:56 reddy Exp $
 *
 * @see Gcc_To_Gdc_Converter
 * @see Gcc_To_Utm_Converter
 * @see Utm_To_Gcc_Converter
 * @see Gdc_To_Gcc_Converter
 * @see Gdc_To_Utm_Converter
 *
 */
class Utm_Coord_3d;
class Gdc_Coord_3d;

class Utm_To_Gdc_Converter
{
private:
    static const double DEGREES_PER_RADIAN;

    static double A,
                  F,
                  C,
                  Eps2,
                  Eps21,
                  Eps25,
                  Con,
                  Con2,
                  EF,
                  Epsp2,
                  Con6,
                  Con24,
                  Con120,
                  Con720,
                  polx2b,
                  polx3b,
                  polx4b,
                  polx5b,
                  conap;

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

//	public static void Init(double a, double f);

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

protected: 
	static void CreateConstants(double a, double f);

   /*
    * Method: Convert
    * 
    * Description: *//**
    *   Performs a single coordinate transform.
    *  
    *   This function will transform a single coordinate. The
    *   input coordinate is provided in the Utm_Coord_3d instance.
    *   The result of the transformation will be put in the
    *   Gdc_Coord_3d instance.
    *
    * @param utm_coord   the input UTM coordinate
    * @param gdc_coord   the output GDC coordinate
    *
    * @return void
    *
    * @since  1.0
    *
    * @see geotransform.coords.Utm_Coord_3d
    * @see geotransform.coords.Gdc_Coord_3d
    *
    */

public: 
	static void Convert(Utm_Coord_3d utm_coord, Gdc_Coord_3d& gdc_coord);

   /*
    * Method: Convert
    * 
    * Description: *//**
    *   Performs multiple coordinate transforms.
    *  
    *   This function will transform any array
    *   of coordinates. The
    *   input coordinate array is provided in the Utm_Coord_3d[] instance.
    *   The results of the transformation will be put into each element
    *   of the Gdc_Coord_3d[] instance. You should have at least as
    *   many entries in the target array as exist in the source array.
    *
    * @param utm   the input array of UTM coordinates
    * @param gdc   the output array of GDC coordinates
    *
    * @return void
    *
    * @since  1.0
    *
    * @see geotransform.coords.Utm_Coord_3d
    * @see geotransform.coords.Gdc_Coord_3d
    *
    */

public: 
	static void Convert(int count, const Utm_Coord_3d utm[], Gdc_Coord_3d gdc[] );

}; // end Utm_To_Gdc_Converter

#endif // !defined(geo_Utm_To_Gdc_Converter)

/*!@file Devices/VectorField.H Vector field class to compute vector
histogram field */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Farhan Baluch
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/VectorField.H $
// $Id: VectorField.H 15272 2012-05-07 20:40:14Z kai $
//

#ifndef VectorField_H_DEFINED
#define VectorField_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Types.H"      // for byte
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Image/vec2.h"

using namespace geom;

    class VectorField : public ModelComponent
    {
        public:
            //! Default constructor; see ModelComponent.H
        VectorField(OptionManager& mgr,
                    const std::string& descrName = "VectorField",
                    const std::string& tagName = "VectorField",
                    const int width=300, const int height=300);

            //!constructor from field
        VectorField(Image<geom::vec2f> afield){
            itsField = afield; itsDims=afield.getDims();
        }


            //! Destructor
        ~VectorField();

            //!getField() -- returns the current vector field.
        Image<geom::vec2f> getField();
      
            //!getGoalField() -- returns the current vector field.
        Image<geom::vec2f> getGoalField();


            /*!updateField() -- update the current field based on some sensor data
              of type Point2D<int> a; where a.i = distance a.j=angle of obstacle
              since this can be pretty implementation specific we keep this virtual
              so that different algorithms can implement their own implementation of
              this method.
            */
        virtual Image<PixRGB<byte> > updateField(
            std::vector<Point2D<float> > sensorData, Point2D<int> robotPos,
            float robotOrient, Point2D<int> goal)
            {
                Image<PixRGB<byte> > emptyImg(1,1,ZEROS);
                return emptyImg;
            }

            //!plotField() -- plot the current vector Field.
        Image<PixRGB<byte> >  plotField(int spacing);

            //!plotGridField() -- plot the current vector Field.
        Image<PixRGB<byte> >  plotGridField(int spacing);

                //!plotGoalField() -- plot the current goal Field.
        Image<PixRGB<byte> >  plotGoalField(int spacing);

            //!getVectorAt() -- get the vector at current robot location
        geom::vec2f getVectorAt(Point2D<int> location);

            //!setVectorAt() -- get the vector at current robot location
        void setVectorAt(Point2D<int> location, geom::vec2f val);

            //!scaleFieldby -- scale vector field by factor
        void scaleFieldBy(float factor);

        //!scaleGoalFieldby -- scale Goal vector field by factor
        void scaleGoalFieldBy(float factor);

            //!make unit length -- set all vectors in vector field to fixed length
        void makeUnitLength();

            //!normalizeTo -- normalize to a max factor
        void normalizeTo(float maxFactor);

          //!rotateField -- rotate the vector field by some angle
        void rotateField(float ang);

            //!rotateGoalField -- rotate the vector field by some angle
        void rotateGoalField(float ang);


        protected:

        Image<geom::vec2f> itsField;
        Image<geom::vec2f> itsGoalField;
        std::vector<Point2D<float> > itsSensorData;
        Point2D<int> itsRobotPos;
        float itsRobotOrientation;
        Point2D<float> itsGoal;

        Dims itsDims;
    };







#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

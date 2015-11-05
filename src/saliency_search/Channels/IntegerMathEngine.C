/*!@file Channels/IntegerMathEngine.C */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/IntegerMathEngine.C $
// $Id: IntegerMathEngine.C 8160 2007-03-21 21:34:16Z rjpeters $
//

#ifndef CHANNELS_INTEGERMATHENGINE_C_DEFINED
#define CHANNELS_INTEGERMATHENGINE_C_DEFINED

#include "Channels/IntegerMathEngine.H"

#include "Channels/ChannelOpts.H"
#include "Component/ModelOptionDef.H"
#include "Image/c_integer_math_ops.h"

IntegerMathEngine::IntegerMathEngine(OptionManager& mgr)
  :
  ModelComponent(mgr, "Integer Math Engine", "IntegerMathEngine"),
  itsScaleBits(&OPT_IntChannelScaleBits, this),
  itsLowPass5(&OPT_IntMathLowPass5, this),
  itsLowPass9(&OPT_IntMathLowPass9, this),
  itsImath()
{

  itsImath.nbits = 30;
  itsImath.low_pass_5_x_dec_x_manybits = &c_intg_low_pass_5_x_dec_x_manybits;
  itsImath.low_pass_5_y_dec_y_manybits = &c_intg_low_pass_5_y_dec_y_manybits;
  itsImath.low_pass_5_x_dec_x_fewbits = &c_intg_low_pass_5_x_dec_x_fewbits;
  itsImath.low_pass_5_y_dec_y_fewbits = &c_intg_low_pass_5_y_dec_y_fewbits;
  itsImath.low_pass_9_x_manybits = &c_intg_low_pass_9_x_manybits;
  itsImath.low_pass_9_y_manybits = &c_intg_low_pass_9_y_manybits;
  itsImath.low_pass_9_x_fewbits = &c_intg_low_pass_9_x_fewbits;
  itsImath.low_pass_9_y_fewbits = &c_intg_low_pass_9_y_fewbits;
  itsImath.x_filter_clean_manybits = &c_intg_x_filter_clean_manybits;
  itsImath.x_filter_clean_fewbits = &c_intg_x_filter_clean_fewbits;
  itsImath.x_filter_clean_small_manybits = &c_intg_x_filter_clean_small_manybits;
  itsImath.x_filter_clean_small_fewbits = &c_intg_x_filter_clean_small_fewbits;
  itsImath.y_filter_clean_manybits = &c_intg_y_filter_clean_manybits;
  itsImath.y_filter_clean_fewbits = &c_intg_y_filter_clean_fewbits;
  itsImath.y_filter_clean_small_manybits = &c_intg_y_filter_clean_small_manybits;
  itsImath.y_filter_clean_small_fewbits = &c_intg_y_filter_clean_small_fewbits;

}

void IntegerMathEngine::paramChanged(ModelParamBase* const param,
                                     const bool valueChanged,
                                     ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  if (param == &itsScaleBits)
    itsImath.nbits = itsScaleBits.getVal();

  else if (param == &itsLowPass5)
    {
      if (itsLowPass5.getVal().compare("lp5std") == 0)
        {
          itsImath.low_pass_5_x_dec_x_manybits = &c_intg_low_pass_5_x_dec_x_manybits;
          itsImath.low_pass_5_y_dec_y_manybits = &c_intg_low_pass_5_y_dec_y_manybits;
          itsImath.low_pass_5_x_dec_x_fewbits = &c_intg_low_pass_5_x_dec_x_fewbits;
          itsImath.low_pass_5_y_dec_y_fewbits = &c_intg_low_pass_5_y_dec_y_fewbits;
        }
      else if (itsLowPass5.getVal().compare("lp5optim") == 0)
        {
          itsImath.low_pass_5_x_dec_x_manybits = &c_intg_low_pass_5_x_dec_x_manybits;
          itsImath.low_pass_5_y_dec_y_manybits = &c_intg_low_pass_5_y_dec_y_manybits;
          itsImath.low_pass_5_x_dec_x_fewbits = &c_intg_low_pass_5_x_dec_x_fewbits_optim;
          itsImath.low_pass_5_y_dec_y_fewbits = &c_intg_low_pass_5_y_dec_y_fewbits_optim;
        }
      else
        {
          LFATAL("Invalid value for --%s; "
                 "valid values are lp5std, lp5optim",
                 itsLowPass5.getOptionDef()->longoptname);
        }

      LINFO("Using lowpass5 algorithm '%s'",
            itsLowPass5.getVal().c_str());
    }

  else if (param == &itsLowPass9)
    {
      if (itsLowPass9.getVal().compare("lp9std") == 0)
        {
          itsImath.low_pass_9_x_manybits = &c_intg_low_pass_9_x_manybits;
          itsImath.low_pass_9_y_manybits = &c_intg_low_pass_9_y_manybits;
          itsImath.low_pass_9_x_fewbits = &c_intg_low_pass_9_x_fewbits;
          itsImath.low_pass_9_y_fewbits = &c_intg_low_pass_9_y_fewbits;
        }
      else if (itsLowPass9.getVal().compare("lp9optim") == 0)
        {
          itsImath.low_pass_9_x_manybits = &c_intg_low_pass_9_x_manybits;
          itsImath.low_pass_9_y_manybits = &c_intg_low_pass_9_y_manybits;
          itsImath.low_pass_9_x_fewbits = &c_intg_low_pass_9_x_fewbits_optim;
          itsImath.low_pass_9_y_fewbits = &c_intg_low_pass_9_y_fewbits_optim;
        }
      else
        {
          LFATAL("Invalid value for --%s; "
                 "valid values are lp9std, lp9optim",
                 itsLowPass9.getOptionDef()->longoptname);
        }

      LINFO("Using lowpass9 algorithm '%s'",
            itsLowPass9.getVal().c_str());
    }

}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
#endif // CHANNELS_INTEGERMATHENGINE_C_DEFINED

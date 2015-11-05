%/*!@file Robots/matlab/localization/getObs.m  */
%
%
%// //////////////////////////////////////////////////////////////////// //
%// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
%// University of Southern California (USC) and the iLab at USC.         //
%// See http://iLab.usc.edu for information about this project.          //
%// //////////////////////////////////////////////////////////////////// //
%// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
%// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
%// in Visual Environments, and Applications'' by Christof Koch and      //
%// Laurent Itti, California Institute of Technology, 2001 (patent       //
%// pending; application number 09/912,225 filed July 23, 2001; see      //
%// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
%// //////////////////////////////////////////////////////////////////// //
%// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
%//                                                                      //
%// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
%// redistribute it and/or modify it under the terms of the GNU General  //
%// Public License as published by the Free Software Foundation; either  //
%// version 2 of the License, or (at your option) any later version.     //
%//                                                                      //
%// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
%// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
%// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
%// PURPOSE.  See the GNU General Public License for more details.       //
%//                                                                      //
%// You should have received a copy of the GNU General Public License    //
%// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
%// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
%// Boston, MA 02111-1307 USA.                                           //
%// //////////////////////////////////////////////////////////////////// //
%//
%// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
%// $HeadURL $
%// $Id $
%//

function [bearing, range, landmarksIDs] = getObs(State, Landmarks_x, Landmarks_y)
  
  xState = 1;
  yState = 2;
  thState = 3;
  
  bearing = zeros(size(Landmarks_x,1), 1);
  range = zeros(size(Landmarks_x,1), 1);
  landmarksIDs = zeros(size(Landmarks_x,1), 1);

  for i=1:size(Landmarks_x,1)
		bearing(i) = atan2(Landmarks_y(i) - State(yState), Landmarks_x(i) - State(xState)) - State(thState);
		range(i) = (Landmarks_y(i) - State(yState))^2 + (Landmarks_x(i) - State(xState))^2;
		range(i) = sqrt(range(i));
		landmarksIDs(i) = i;


		bearing(i) = bearing(i) + ((2 - (2*2*randn))*pi/180);
		range(i) = range(i) + (1 - (1*2*randn));

    %bearing(i) = pi_to_pi(bearing(i));

  end





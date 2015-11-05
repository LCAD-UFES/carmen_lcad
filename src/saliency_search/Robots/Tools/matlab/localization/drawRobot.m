%/*!@file Robots/matlab/localization/drawRobot.m  */
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


function drawRobot(State, col)

	xState = 1;
	yState = 2;
	thState = 3;

  ShiftTheta = 0;
	p=0.02; % percentage of axes size 
	a=axis;
	l1=(a(2)-a(1))*p;
	l2=(a(4)-a(3))*p;
	P=[-1 1 0 -1; -1 -1 3 -1];%basic triangle
	theta = State(thState)-pi/2+ShiftTheta;%rotate to point along x axis (theta = 0)
	c=cos(theta);
	s=sin(theta);
	P=[c -s; s c]*P; %rotate by theta
	P(1,:)=P(1,:)*l1+State(xState); %scale and shift to x
	P(2,:)=P(2,:)*l2+State(yState);
	H = plot(P(1,:),P(2,:),col,'LineWidth',0.1);% draw
	plot(State(xState),State(yState),sprintf('%s+',col));

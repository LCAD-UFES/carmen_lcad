%/*!@file Robots/matlab/localization/ekf.m  */
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


%Some constents
xState = 1;
yState = 2;
thState = 3;

TrueState = zeros(3,1);
CurrentState = zeros(3,1);
CurrentCov = diag([0.1 0.1 2*pi/180]).^2;

nLandmarks = 10;
Landmarks_x = zeros(nLandmarks,1);
Landmarks_y = zeros(nLandmarks,1);
for i=1:nLandmarks
	Landmarks_x(i) = 100 - 200*rand;
	Landmarks_y(i) = 100 - 200*rand;
end

figure
for i=1:100
  clf
  axis([-300 300 -300 300]);
  hold on
  %Get a control
  u_f = 2; 
  u_th = 0; %.02;

  %Simulate the robot
  TrueState = moveRobot(TrueState, u_f, u_th);

  %Probogate the state and compute the veriance
  %New state
  State_hat = zeros(3,1);
  State_hat(xState)  = CurrentState(xState) + u_f*cos(CurrentState(thState));
  State_hat(yState)  = CurrentState(yState) + u_f*sin(CurrentState(thState));
  State_hat(thState) = CurrentState(thState) + u_th;

  %New coveriance
  A = [1 0 -u_f*sin(CurrentState(thState));
       0 1  u_f*cos(CurrentState(thState));
       0 0  1];

  B = [cos(CurrentState(thState)) 0;
       sin(CurrentState(thState)) 0;
       0	                         1];

  G = diag([1; 1]); %The input noise, Gamma

  Cov_hat = A * CurrentCov * A' + B*G*B';


  %Get observation
  [bearings, ranges] = getObs(TrueState, Landmarks_x, Landmarks_y);

  %Update the state for each landmark
  Q = diag([0.1 0.1 1]).^2;

  State_hat
  for li=1:size(bearings,1)
	  %Predict the bearing and range for this landmark
	  belifeRange = (Landmarks_y(li) - State_hat(yState))^2 + ...
	                (Landmarks_x(li) - State_hat(xState))^2;
	  belifeRange = sqrt(belifeRange);
	  belifeBearing = atan2(Landmarks_y(li) - State_hat(yState), ...
	                        Landmarks_x(li) - State_hat(xState)) ...
			       	+ State_hat(thState);

	  %Compute the mesuement jacobian and the kalman innovation
	  H = [ (State_hat(xState) - Landmarks_x(li)) / belifeRange  ...
	      (State_hat(yState) - Landmarks_y(li)) / belifeRange ...
	      0 ...
	      ; ...
	      (Landmarks_y(li) - State_hat(yState)) / (belifeRange^2) ...
	      -1*(Landmarks_x(li) - State_hat(xState)) / (belifeRange^2)  ...
	      1;
	      0 0 0];
	  Innov = [ranges(li) - belifeRange ; bearings(li) - belifeBearing ; 1];

	  %Update the state and coveriance
	  S = H*Cov_hat*H' + Q;
	  K = Cov_hat*H'*inv(S);

	  State_hat = State_hat + K*Innov;
	  Cov_hat = Cov_hat - K*S*K';

	  %Plot each landmark mesurment
	  range = ranges(li,1);
	  landmarkX = State_hat(xState) - (range*cos(bearings(li,1) - State_hat(thState) + pi));
	  landmarkY = State_hat(yState) - (range*sin(bearings(li,1) - State_hat(thState) + pi));
	  line([State_hat(xState) landmarkX], [State_hat(yState) landmarkY]);

	  %drawRobot(TrueState, 'g');
	  %drawLandmarks(Landmarks_x, Landmarks_y);
	  %drawRobot(State_hat, 'r');
  end

  CurrentState = State_hat
  CurrentCov = Cov_hat
  drawRobot(TrueState, 'g');
  drawLandmarks(Landmarks_x, Landmarks_y);
  drawRobot(CurrentState, 'r');
  h = drawEllipse(State_hat, Cov_hat,100);
  if(~isempty(h))
		set(h,'color','r');
  end;
  pause



  %Draw
end

  

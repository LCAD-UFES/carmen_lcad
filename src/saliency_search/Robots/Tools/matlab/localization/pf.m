%/*!@file Robots/matlab/localization/pf.m  */
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

%Our state
TrueState = zeros(3,1);
CurrentState = zeros(3,1);
CurrentCov = diag([0.1 0.1 2*pi/180]).^2;

%The landmarks
nLandmarks = 10;
Landmarks_x = zeros(nLandmarks,1);
Landmarks_y = zeros(nLandmarks,1);
for i=1:nLandmarks
	Landmarks_x(i) = 100 - 200*rand;
	Landmarks_y(i) = 100 - 200*rand;
end

Particles = zeros(100,4);
%Initalize particles with a uniform values
Particles(:,4) = 1/size(Particles,1);

%Place the particlues at diffrent states
for i=1:size(Particles,1)
  Particles(i,xState) = 20-40*rand;
  Particles(i,yState) = 20-40*rand;
  Particles(i,thState) = 15*rand*pi/180;
end


figure
for i=1:100
  clf
  axis([-200 200 -200 200]);
  hold on
  %Get a control
  u_f = 2; 
  u_th = 0; %.02;

  %Simulate the robot
  TrueState = moveRobot(TrueState, u_f, u_th);


  %%Probogate the state and compute the veriance
  %%New state
  for i=1:size(Particles,1)
    Particles(i,xState)  = Particles(i,xState) + u_f*cos(Particles(i,thState)) + randn*15;
    Particles(i,yState)  = Particles(i,yState) + u_f*sin(Particles(i,thState)) + randn*15;;
    Particles(i,thState) = Particles(i,thState) + u_th + randn*20*pi/180;
  end

  [bearings, ranges] = getObs(TrueState, Landmarks_x, Landmarks_y);
  for i=1:size(Particles,1)
    %Calculate the likelihood
    State_hat = Particles(i,:);
    prob = 1;
    for li = 1:size(bearings,1)
          belifeRange = (Landmarks_y(li) - State_hat(yState))^2 + ...
              (Landmarks_x(li) - State_hat(xState))^2;
          belifeRange = sqrt(belifeRange);

          belifeBearing = atan2(Landmarks_y(li) - State_hat(yState), ...
              Landmarks_x(li) - State_hat(xState)) ...
              + State_hat(thState);

					Innov= [ranges(li)-belifeRange; bearings(li)-belifeBearing];
					Q = diag([5 5*pi/180]).^2;
					prob = prob * exp(-0.5*Innov'*inv(Q)*Innov);

          %prob = prob * exp((-0.5*(ranges(li)-belifeRange)^2)/Q(1,1));
          %prob = prob * exp((-0.5*(bearings(li)-belifeBearing)^2)/Q(2,2));
    end;

    Particles(i,4) = Particles(i,4) * prob;
  end

  %Normalize
  Particles(:,4) = Particles(:,4) ./ sum(Particles(:,4));

  %Compute an estimate of effective particles
  Neff = 1/(sum(Particles(:,4).^2))


  %Resample based on particles-files.ppt
  %Generate CDF
  NewParticles = zeros(size(Particles));
  CDF = cumsum(Particles(:,4));

  i = 1; 
  u = rand * 1/size(Particles,1);
  for j=1:size(Particles,1)
	  while(u > CDF(i))
		  i = i + 1;
		end
		NewParticles(j,:) = Particles(i,:);
		NewParticles(:,4) = 1/size(Particles,1);
		u = u + 1/size(Particles,1);
	end
	Particles = NewParticles;

  %Show results
  plot3(Particles(:,xState), Particles(:,yState), Particles(:,4), '.r');
  %plot(Particles(:,xState), Particles(:,yState), '.r');
  drawLandmarks(Landmarks_x, Landmarks_y);
  drawRobot(TrueState, 'g');
	%Draw the max particle
  %[x i] = max(Particles(:,4));
  %drawRobot(Particles(i,:), 'r');

	drawRobot(mean(Particles), 'r');
  pause
end

  

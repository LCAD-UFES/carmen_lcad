%/*!@file Robots/matlab/localization/pf_fastslam.m  */
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

clear
%Some constents
xState = 1;
yState = 2;
thState = 3;

%Our state
TrueState = [0; 0; 0];

%The landmarks
nLandmarks = 10;
Landmarks_x = zeros(nLandmarks,1);
Landmarks_y = zeros(nLandmarks,1);
for i=1:nLandmarks
  Landmarks_x(i) = 400 - 800*rand;
  Landmarks_y(i) = 400 - 800*rand;
end

nParticles = 100;

for i=1:nParticles
  Particles{i}.pos = [0; 0; 0]; %[2-4*rand; 2-4*rand; 0];
  Particles{i}.landmarksPos = [];
  Particles{i}.landmarksCov = [];
  Particles{i}.prob = 1/nParticles;
end

R = diag([5*pi/180]).^2;

for i=1:1000
  clf
  axis([-400 400 -400 400]);
  hold on 
  %Get a control
  u_f = 2;
  u_th = 0.02;

  %Simulate the robot
  TrueState = moveRobot(TrueState, u_f, u_th);

  [bearings, ranges, landmarksIDs] = getObs(TrueState, Landmarks_x, Landmarks_y);

  drawLandmarks(Landmarks_x, Landmarks_y, 'gx');
  drawRobot(TrueState, 'g');

  for k=1:nParticles
    %Sample pose from the motion model
    currentPos = Particles{k}.pos;
	 V = 10 - rand*20;
	 G = (20 - 40*rand)*pi/180;
    currentPos(xState)  = currentPos(xState) + (V+u_f)*cos(currentPos(thState));
    currentPos(yState)  = currentPos(yState) + (V+u_f)*sin(currentPos(thState));
    currentPos(thState) = currentPos(thState) + (u_th +G);
    
    %Look for this landmark in our state map
    for li=1:size(landmarksIDs,1)

      landmarkPos = [];
      landmarkCov = [];

      if (size(Particles{k}.landmarksPos,2) >= li)
        landmarkPos = Particles{k}.landmarksPos{li};
        landmarkCov = Particles{k}.landmarksCov{li};
      end

      
      %if this is the first time we are seeing this landmark
      %Then add it to our current map
      if (isempty(landmarkPos))
        landmark_id = landmarksIDs(li);

        %initalize landmark mean as u_j = g-1(z,x)
        landmark_x = currentPos(xState) + (200*cos(bearings(li) + currentPos(thState)));
        landmark_y = currentPos(yState) + (200*sin(bearings(li) + currentPos(thState)));
        landmarkPos = [landmark_x, landmark_y];

        %calculate jacobian G = g'(x,u_j) wrt u
        range = (landmarkPos(2) - currentPos(yState))^2 + ...
                (landmarkPos(1) - currentPos(xState))^2;
        range = sqrt(range);

        G = [ (landmarkPos(1) - currentPos(xState))/range (landmarkPos(2) - currentPos(yState))/range ;
              (currentPos(yState) - landmarkPos(2))/range^2   (currentPos(xState) - landmarkPos(1))/range^2 ];

        %initalize coveriance
		  RVar = diag([50 5*pi/180]).^2; 
        landmarkCov = inv(G)*RVar*inv(G)';
        %landmarkCov = diag([5 5]);
        iw = 1/size(Particles,2); %defualt importance weight

      else
        %Mesurment prediction z=g(u,x)
        
        dx = landmarkPos(1) - currentPos(xState);
        dy = landmarkPos(2) - currentPos(yState);
        d2 = dx^2 + dy^2;
        d = sqrt(d2);

        belifeBearing = atan2(dy,dx) - currentPos(thState);

        G = [-dy/d2, dx/d2];

        Innov = [bearings(li) - belifeBearing];

        PHt = landmarkCov*G';
        S = G*PHt + R;
        
        S = (S+S')*0.5; %make symmetric
        SChol = chol(S);
        
        SCholInv = inv(SChol);
        W1 = PHt * SCholInv;
        K = W1 * SCholInv';
        
        
        Q = G*landmarkCov*G' + R;
        
        landmarkPos = landmarkPos + (K*Innov)';
        landmarkCov = landmarkCov - W1*W1';

        iw = exp(-0.5*Innov'*inv(Q)*Innov) + 0.001; %/(2*pi*sqrt(det(Q))); 

      end
    
       %drawEllipse(landmarkPos', landmarkCov, 3);
       %plot(landmarkPos(1), landmarkPos(2), 'b*');
       %Update the state and map 
       Particles{k}.pos = currentPos;
       Particles{k}.landmarksPos{li} = landmarkPos;
       Particles{k}.landmarksCov{li} = landmarkCov;
       Particles{k}.prob = Particles{k}.prob * iw;
       plot(currentPos(1), currentPos(2), '.b');
    end
  end
 
  %Normalize
  pSum = 0;
  for i=1:size(Particles,2)
    pSum = pSum + Particles{i}.prob;
  end

  for i=1:size(Particles,2)
    Particles{i}.prob = Particles{i}.prob/pSum;
  end

  %%Compute an estimate of effective particles
  %Neff = 1/(sum(Particles(:,4).^2))

  %Show results
  [robotPos, Map] = getRobotAndMapState(Particles);
  drawLandmarks(Map(:,1), Map(:,2), 'ro');
  drawRobot(robotPos, 'r');
  drawnow
  %pause

  %Resample based on particles-files.ppt
  %Generate CDF
  NewParticles = [];
  CDF = zeros(size(Particles,2),1);
  CDF(1) = Particles{1}.prob;
  for i=2:size(Particles,2)
    CDF(i) = CDF(i-1) + Particles{i}.prob;
  end

  i = 1; 
  u = rand * 1/size(Particles,2);
  for j=1:size(Particles,2)
    while(u > CDF(i))
      i = i + 1;
    end
    NewParticles{j} = Particles{i};
    NewParticles{j}.prob = 1/size(Particles,2);
    u = u + 1/size(Particles,2);
  end
  Particles = NewParticles;



  %plot3(Particles(:,xState), Particles(:,yState), Particles(:,4), '.r');
  %%plot(Particles(:,xState), Particles(:,yState), '.r');
  %drawLandmarks(LandmarksState(:,2), LandmarksState(:,3), 'ro');
  %%Draw the max particle
  %%[x i] = max(Particles(:,4));
  %%drawRobot(Particles(i,:), 'r');

  %drawRobot(mean(Particles), 'r');
end

  

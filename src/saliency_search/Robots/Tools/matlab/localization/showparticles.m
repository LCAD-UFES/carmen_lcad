

for i=1:nParticles
  [i Particles{i}.prob]
  clf
  axis([-200 200 -200 200]);
  hold on
  
  
  drawLandmarks(Landmarks_x, Landmarks_y, 'gx');
  drawRobot(TrueState, 'g');
  
  State = Particles{i}.pos;
	Map = cell2mat(Particles{i}.landmarksPos(:));

  drawLandmarks(Map(:,1), Map(:,2), 'ro');
  drawRobot(State, 'r');
  pause;
end


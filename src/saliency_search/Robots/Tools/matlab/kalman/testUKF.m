
%Feature estimate
featureState.pos = [640/2; 480/2; 0; 0; 0];

featureState.cov = zeros(5,5);
featureState.cov(1,1) = 150*150;
featureState.cov(2,2) = 150*150;
featureState.cov(3,3) = (1*pi/180)*(1*pi/180);
featureState.cov(4,4) = 0.01*0.01;
featureState.cov(5,5) = (1*pi/180)*(1*pi/180);

posVar = 10;
angVar= 5*pi/180;
tranVelVar= 0.1;
angVelVar= 0.1*pi/180;
R = zeros(5,5);
R(1,1) = posVar*posVar;
R(2,2) = posVar*posVar;
R(3,3) = angVar*angVar;
R(4,4) = tranVelVar*tranVelVar;
R(5,5) = angVelVar*angVelVar;


Q = zeros(2,2);
Q(1,1) = 10*10;
Q(2,2) = (5*pi/180)*(5*pi/180);


%How the feature evolves
f=@(x)[
x(1) - x(4)/(x(5)+eps)*sin(x(3)) + x(4)/(x(5)+eps)*sin(x(3)+x(5)); %position
x(2) + x(4)/(x(5)+eps)*cos(x(3)) - x(4)/(x(5)+eps)*cos(x(3)+x(5)); %position
x(3) + x(5); %Angle
x(4); %transitional velocity
x(5) % Rotational velocity
]; %state equation
%h=@(x)[x(1)*cos(x(2)); x(1)*sin(x(2))];
h=@(x)[sqrt(x(1)^2 + x(2)^2); atan2(x(2),x(1))];
%h=@(x)[x(1); x(2)];


particlePosition = [640/2; 480/2-100];
theta = 0;

history = zeros(360, 3*2);
for itr=1:360
  %Particle Simulation
  rangeNoise = ceil(rand*8)-4;
  angNoise = ceil(rand*4)-2;
  dR = 5+rangeNoise;
  dA = (2+angNoise)*pi/180;
  xc = particlePosition(1) - dR/dA*sin(theta);
  yc = particlePosition(2) + dR/dA*cos(theta);

  particlePosition(1) = xc + dR/dA*sin(theta + dA);
  particlePosition(2) = yc - dR/dA*cos(theta + dA);
  theta = theta + dA;

  %Compute measurements
  z = zeros(2,1);
  z(1) = sqrt(particlePosition(1)^2 + particlePosition(2)^2);
  z(2) = atan2(particlePosition(2), particlePosition(1));
  %Add the sensor noise
  z(1) = z(1) + ceil(rand*10)-5;
  z(2) = z(2) + (ceil(rand*3)-1.5)*pi/180; 

  [featureState.pos, featureState.cov, muPredicted, covPredicted] = ukf2(f,featureState.pos,
                                             featureState.cov, h, z,
                                             Q,R);

  sensorPos=[z(1)*cos(z(2)); z(1)*sin(z(2))];

  history(itr, 1) = particlePosition(1);
  history(itr, 2) = particlePosition(2);
  history(itr, 3) = sensorPos(1);
  history(itr, 4) = sensorPos(2);
  history(itr, 5) = featureState.pos(1);
  history(itr, 6) = featureState.pos(2);

  clf; hold on
  %Show the particle
  plot(particlePosition(1), particlePosition(2), 'gx');

  %Show the sensor 
  plot(sensorPos(1), sensorPos(2), 'kx');

  %Show our predicted feature position
  plot(muPredicted(1), muPredicted(2), 'bx');
  plotcov2(muPredicted(1:2), covPredicted(1:2,1:2), 'b');

  %Show our current feature position
  plot(featureState.pos(1), featureState.pos(2), 'r.');
  plotcov2(featureState.pos(1:2), featureState.cov(1:2,1:2), 'r');

  plot(history(:,1), history(:,2), 'g');
  plot(history(:,3), history(:,4), 'b');
  plot(history(:,5), history(:,6), 'r');

  axis([0 640 0 480]);
  pause;
end


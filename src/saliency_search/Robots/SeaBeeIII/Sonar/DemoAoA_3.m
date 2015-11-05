%% This script demos the simple phase difference angle-of-arrival (AoA)
%% calculation used by SeaBeaIII to estimate the direction of the sonar pinger.
%% This demo considers AoA estimation using three hydrophones.
%% Zero degrees is defined as directly ahead (normal to sensor plane).

%% Initialize Environment
clear all; close all;

%% Global Vars
N.samples = 512;
N.fft = 512;
SPEED_SOUND_WATER = 1482; % [m/s]

SIGNAL_FREQ = 30e3;
SAMPLING_FREQ = 104706; % [Hz]
WAVELENGTH = SPEED_SOUND_WATER / SIGNAL_FREQ;

SENSOR_SPACING = 0.020; % [m] (between each possible paring)

%% Target Details
target.distance = 100; % [m]

%% File I/O
testDataFile = fopen('TestData3.H', 'w+');
fprintf(testDataFile, '#ifndef SEABEA_SONAR_TESTDATA_H\n#define SEABEA_SONAR_TESTDATA_H\n\n');

%% Sensor Definitions
R = 1/3 * sqrt(3) * SENSOR_SPACING;
sensor1.angle = 150 * pi / 180;
sensor1.x = R * cos(sensor1.angle);
sensor1.y = R * sin(sensor1.angle);
sensor2.angle = 30 * pi / 180;
sensor2.x = R * cos(sensor2.angle);
sensor2.y = R * sin(sensor2.angle);
sensor3.angle = 270 * pi / 180;
sensor3.x = R * cos(sensor3.angle);
sensor3.y = R * sin(sensor3.angle);

figure(5); hold on;
    xlabel('Actual Angle');
    ylabel('Sqaured Error');
    title('Comparison of Sensor Pair Squared Errors');

div = 1;
x = -180:1/div:178;
y = zeros(1,length(x));
phase = zeros(3,length(x));
squared_error = zeros(1,length(x));
for i=1:length(x)
  target.angle = i / div     * pi / 180 + pi / 2; % [rad]
  sensor1.distance = sqrt((target.distance*cos(target.angle) - sensor1.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor1.y)^2);
  sensor1.delta = mod(sensor1.distance, WAVELENGTH);
  sensor1.phase = sensor1.delta / WAVELENGTH * 2 * pi;

  sensor2.distance = sqrt((target.distance*cos(target.angle) - sensor2.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor2.y)^2);
  sensor2.delta = mod(sensor2.distance, WAVELENGTH);
  sensor2.phase = sensor2.delta / WAVELENGTH * 2 * pi;

  sensor3.distance = sqrt((target.distance*cos(target.angle) - sensor3.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor3.y)^2);
  sensor3.delta = mod(sensor3.distance, WAVELENGTH);
  sensor3.phase = sensor3.delta / WAVELENGTH * 2 * pi;
  
  %sensor1, sensor2, sensor3

  %% Generate Sample Data
  sensor1.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor1.phase);
  sensor2.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor2.phase);
  sensor3.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor3.phase);

  %% FFT
  data.fft1 = fft(sensor1.observation, N.fft);
  data.fft2 = fft(sensor2.observation, N.fft);
  data.fft3 = fft(sensor3.observation, N.fft);

  [data.fft1_maxValue data.fft1_maxIndex] = max(data.fft1(1:N.fft/2));
  [data.fft2_maxValue data.fft2_maxIndex] = max(data.fft2(1:N.fft/2));
  [data.fft3_maxValue data.fft3_maxIndex] = max(data.fft3(1:N.fft/2));

  data.phase1 = atan2(imag(data.fft1(data.fft1_maxIndex)), real(data.fft1(data.fft1_maxIndex)));
  data.phase2 = atan2(imag(data.fft2(data.fft2_maxIndex)), real(data.fft2(data.fft2_maxIndex)));
  data.phase3 = atan2(imag(data.fft3(data.fft3_maxIndex)), real(data.fft3(data.fft3_maxIndex)));

  %% Phase Differences
  delta = zeros(1,3); % (1 - 2), (2 - 3), (3 - 1)
  delta(1) = data.phase2 - data.phase1;
  delta(2) = data.phase3 - data.phase2;
  delta(3) = data.phase1 - data.phase3;

  phase(1,i) = delta(1);
  phase(2,i) = delta(2);
  phase(3,i) = delta(3);

  phase_difference = delta(1);
  
  if delta(3) > delta(2)
    phase_difference = -1 * delta(1) + sign(delta(1)) * 2 * pi;
  end

  y(i) = phase_difference * ((WAVELENGTH / 2) / SENSOR_SPACING) * 180 / pi / 2;
  squared_error(i) = (x(i) - y(i))^2;
  %[x(i) delta y(i)]
end

figure(1);
  subplot(2,1,1);
  plot(x,y)
  line([-180 180], [-180 180], 'Color', 'green', 'LineStyle', '--');
  xlabel('Actual Angle');
  ylabel('Estimated Angle');
  title('Angle Estimation: Sensor Pair (1,2)');

figure(1);
  subplot(2,1,2);
  plot(x,squared_error);
  xlabel('Actual Angle');
  ylabel('Sqaured Error');
  title('Squared Error: Sensor Pair (1,2)');

figure(10);
  subplot(4,1,1);
  plot(x,phase(1,:))
  subplot(4,1,2);
  plot(x,phase(2,:))
  subplot(4,1,3);
  plot(x,phase(3,:))

figure(5); hold on;
  plot(x,squared_error, 'r');

y = zeros(1,length(x));
phase = zeros(3,length(x));
squared_error = zeros(1,length(x));
for i=1:length(x)
  target.angle = i / div     * pi / 180 + pi / 2; % [rad]
  sensor1.distance = sqrt((target.distance*cos(target.angle) - sensor1.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor1.y)^2);
  sensor1.delta = mod(sensor1.distance, WAVELENGTH);
  sensor1.phase = sensor1.delta / WAVELENGTH * 2 * pi;

  sensor2.distance = sqrt((target.distance*cos(target.angle) - sensor2.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor2.y)^2);
  sensor2.delta = mod(sensor2.distance, WAVELENGTH);
  sensor2.phase = sensor2.delta / WAVELENGTH * 2 * pi;

  sensor3.distance = sqrt((target.distance*cos(target.angle) - sensor3.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor3.y)^2);
  sensor3.delta = mod(sensor3.distance, WAVELENGTH);
  sensor3.phase = sensor3.delta / WAVELENGTH * 2 * pi;
  
  %sensor1, sensor2, sensor3

  %% Generate Sample Data
  sensor1.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor1.phase);
  sensor2.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor2.phase);
  sensor3.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor3.phase);

  %% FFT
  data.fft1 = fft(sensor1.observation, N.fft);
  data.fft2 = fft(sensor2.observation, N.fft);
  data.fft3 = fft(sensor3.observation, N.fft);

  [data.fft1_maxValue data.fft1_maxIndex] = max(data.fft1(1:N.fft/2));
  [data.fft2_maxValue data.fft2_maxIndex] = max(data.fft2(1:N.fft/2));
  [data.fft3_maxValue data.fft3_maxIndex] = max(data.fft3(1:N.fft/2));

  data.phase1 = atan2(imag(data.fft1(data.fft1_maxIndex)), real(data.fft1(data.fft1_maxIndex)));
  data.phase2 = atan2(imag(data.fft2(data.fft2_maxIndex)), real(data.fft2(data.fft2_maxIndex)));
  data.phase3 = atan2(imag(data.fft3(data.fft3_maxIndex)), real(data.fft3(data.fft3_maxIndex)));

  %% Phase Differences
  delta = zeros(1,3); % (1 - 2), (2 - 3), (3 - 1)
  delta(1) = data.phase2 - data.phase1;
  delta(2) = data.phase3 - data.phase2;
  delta(3) = data.phase1 - data.phase3;

  phase(1,i) = delta(1);
  phase(2,i) = delta(2);
  phase(3,i) = delta(3);

  phase_difference = delta(2);
  
  if delta(1) > delta(3) || (delta(1) > 0 && delta(3) > 0)
    phase_difference = -1 * delta(2) +  2 * pi;
  end

  y(i) = (phase_difference - 4 / 3 * pi) * ((WAVELENGTH / 2) / SENSOR_SPACING) * 180 / pi / 2;
  squared_error(i) = (x(i) - y(i))^2;
  %[x(i) delta y(i)]
end


figure(2);
  subplot(2,1,1);
  plot(x,y)
  line([-180 180], [-180 180], 'Color', 'green', 'LineStyle', '--');
  xlabel('Actual Angle');
  ylabel('Estimated Angle');
  title('Angle Estimation: Sensor Pair (2,3)');

figure(2);
  subplot(2,1,2);
  plot(x,squared_error);
  xlabel('Actual Angle');
  ylabel('Sqaured Error');
  title('Squared Error: Sensor Pair (2,3)');

figure(5); hold on;
    plot(x,squared_error, 'g');

y = zeros(1,length(x));
phase = zeros(3,length(x));
squared_error = zeros(1,length(x));
for i=1:length(x)
  target.angle = i / div     * pi / 180 + pi / 2; % [rad]
  sensor1.distance = sqrt((target.distance*cos(target.angle) - sensor1.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor1.y)^2);
  sensor1.delta = mod(sensor1.distance, WAVELENGTH);
  sensor1.phase = sensor1.delta / WAVELENGTH * 2 * pi;

  sensor2.distance = sqrt((target.distance*cos(target.angle) - sensor2.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor2.y)^2);
  sensor2.delta = mod(sensor2.distance, WAVELENGTH);
  sensor2.phase = sensor2.delta / WAVELENGTH * 2 * pi;

  sensor3.distance = sqrt((target.distance*cos(target.angle) - sensor3.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor3.y)^2);
  sensor3.delta = mod(sensor3.distance, WAVELENGTH);
  sensor3.phase = sensor3.delta / WAVELENGTH * 2 * pi;
  
  %sensor1, sensor2, sensor3

  %% Generate Sample Data
  sensor1.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor1.phase);
  sensor2.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor2.phase);
  sensor3.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor3.phase);

  %% FFT
  data.fft1 = fft(sensor1.observation, N.fft);
  data.fft2 = fft(sensor2.observation, N.fft);
  data.fft3 = fft(sensor3.observation, N.fft);

  [data.fft1_maxValue data.fft1_maxIndex] = max(data.fft1(1:N.fft/2));
  [data.fft2_maxValue data.fft2_maxIndex] = max(data.fft2(1:N.fft/2));
  [data.fft3_maxValue data.fft3_maxIndex] = max(data.fft3(1:N.fft/2));

  data.phase1 = atan2(imag(data.fft1(data.fft1_maxIndex)), real(data.fft1(data.fft1_maxIndex)));
  data.phase2 = atan2(imag(data.fft2(data.fft2_maxIndex)), real(data.fft2(data.fft2_maxIndex)));
  data.phase3 = atan2(imag(data.fft3(data.fft3_maxIndex)), real(data.fft3(data.fft3_maxIndex)));

  %% Phase Differences
  delta = zeros(1,3); % (1 - 2), (2 - 3), (3 - 1)
  delta(1) = data.phase2 - data.phase1;
  delta(2) = data.phase3 - data.phase2;
  delta(3) = data.phase1 - data.phase3;

  phase(1,i) = delta(1);
  phase(2,i) = delta(2);
  phase(3,i) = delta(3);

  phase_difference = delta(3);
  
  if delta(2) > delta(1) || (delta(1) < 0 && delta(2) < 0)
    phase_difference = -1 * delta(3) - 2 * pi;
  end

  y(i) = (phase_difference + 4 / 3 * pi) * ((WAVELENGTH / 2) / SENSOR_SPACING) * 180 / pi / 2;
  squared_error(i) = (x(i) - y(i))^2;
  %[x(i) delta y(i)]
end

figure(3);
  subplot(2,1,1);
  plot(x,y)
  line([-180 180], [-180 180], 'Color', 'green', 'LineStyle', '--');
  xlabel('Actual Angle');
  ylabel('Estimated Angle');
  title('Angle Estimation: Sensor Pair (3,1)');

figure(3);
  subplot(2,1,2);
  plot(x,squared_error);
  xlabel('Actual Angle');
  ylabel('Sqaured Error');
  title('Squared Error: Sensor Pair (3,1)');

figure(5); hold on;
    plot(x,squared_error, 'b');

y = zeros(1,length(x));
phase = zeros(3,length(x));
squared_error = zeros(1,length(x));
for i=1:length(x)
  target.angle = i / div     * pi / 180 + pi / 2; % [rad]
  sensor1.distance = sqrt((target.distance*cos(target.angle) - sensor1.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor1.y)^2);
  sensor1.delta = mod(sensor1.distance, WAVELENGTH);
  sensor1.phase = sensor1.delta / WAVELENGTH * 2 * pi;

  sensor2.distance = sqrt((target.distance*cos(target.angle) - sensor2.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor2.y)^2);
  sensor2.delta = mod(sensor2.distance, WAVELENGTH);
  sensor2.phase = sensor2.delta / WAVELENGTH * 2 * pi;

  sensor3.distance = sqrt((target.distance*cos(target.angle) - sensor3.x)^2 + ...
                          (target.distance*sin(target.angle) - sensor3.y)^2);
  sensor3.delta = mod(sensor3.distance, WAVELENGTH);
  sensor3.phase = sensor3.delta / WAVELENGTH * 2 * pi;
  
  %sensor1, sensor2, sensor3

  %% Generate Sample Data
  sensor1.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor1.phase);
  sensor2.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor2.phase);
  sensor3.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor3.phase);

  %% FFT
  data.fft1 = fft(sensor1.observation, N.fft);
  data.fft2 = fft(sensor2.observation, N.fft);
  data.fft3 = fft(sensor3.observation, N.fft);

  [data.fft1_maxValue data.fft1_maxIndex] = max(data.fft1(1:N.fft/2));
  [data.fft2_maxValue data.fft2_maxIndex] = max(data.fft2(1:N.fft/2));
  [data.fft3_maxValue data.fft3_maxIndex] = max(data.fft3(1:N.fft/2));

  data.phase1 = atan2(imag(data.fft1(data.fft1_maxIndex)), real(data.fft1(data.fft1_maxIndex)));
  data.phase2 = atan2(imag(data.fft2(data.fft2_maxIndex)), real(data.fft2(data.fft2_maxIndex)));
  data.phase3 = atan2(imag(data.fft3(data.fft3_maxIndex)), real(data.fft3(data.fft3_maxIndex)));

  %% Phase Differences
  delta = zeros(1,3); % (1 - 2), (2 - 3), (3 - 1)
  delta(1) = data.phase2 - data.phase1;
  delta(2) = data.phase3 - data.phase2;
  delta(3) = data.phase1 - data.phase3;

  phase(1,i) = delta(1);
  phase(2,i) = delta(2);
  phase(3,i) = delta(3);

  crit = abs(delta); %delta; %abs(delta); %mod(delta, pi);

  % Determine Minimum Phase Difference for Sensor Pair Selection
  [phase_difference best_phase_difference_index] = min(crit);
  phase_difference = delta(best_phase_difference_index);

  best_phase_difference_index
  pair_index(i) = best_phase_difference_index;
  switch best_phase_difference_index
    case 1 % 1 - 2
      if delta(3) > delta(2)
        phase_difference = -1 * delta(1) + sign(delta(1)) * 2 * pi;
      end
      y(i) = phase_difference * ((WAVELENGTH / 2) / SENSOR_SPACING) * 180 / pi / 2;
    case 2 % 2 - 3
      if delta(1) > delta(3)
        phase_difference = -1 * delta(2) + 2 * pi;
      end
      y(i) = (phase_difference - 4 / 3 * pi) * ((WAVELENGTH / 2) / SENSOR_SPACING) * 180 / pi / 2;
    case 3 % 3 - 1
      if delta(2) > delta(1)
        phase_difference = -1 * delta(3) - 2 * pi;
      end
      y(i) = (phase_difference + 4 / 3 * pi) * ((WAVELENGTH / 2) / SENSOR_SPACING) * 180 / pi / 2;
    otherwise
      disp('ERROR!!! This case should never occur')
  end

  %y(i) = phase_difference * ((WAVELENGTH / 2) / SENSOR_SPACING) * 180 / pi / 2;
  [x(i) delta y(i)]
  squared_error(i) = (x(i) - y(i))^2;
  
  ANGLE = -20;
  if i == ANGLE + 180 + 1
    fprintf(testDataFile, 'double data1[512] = {');
    for j=1:N.samples-1
        fprintf(testDataFile, '%3.15f, ', sensor1.observation(j));
    end
    fprintf(testDataFile, '%3.15f};\n', sensor1.observation(N.samples));

    fprintf(testDataFile, 'double data2[512] = {');
    for j=1:N.samples-1
        fprintf(testDataFile, '%3.15f, ', sensor2.observation(j));
    end
    fprintf(testDataFile, '%3.15f};\n', sensor2.observation(N.samples));

    fprintf(testDataFile, 'double data3[512] = {');
    for j=1:N.samples-1
        fprintf(testDataFile, '%3.15f, ', sensor3.observation(j));
    end
    fprintf(testDataFile, '%3.15f};\n', sensor3.observation(N.samples));

    fprintf(testDataFile, '\n#endif // SEABEA_SONAR_TESTDATA_H Defined\n\n');

    fclose(testDataFile);
    best_phase_difference_index
    [x(i) delta y(i)]
  end
end

figure(10);
  subplot(4,1,4)
  plot(x,pair_index);


figure(4);
  subplot(2,1,1);
  plot(x,y)
  line([-180 180], [-180 180], 'Color', 'green', 'LineStyle', '--');
  line([-180 180], [-180 180], 'Color', 'green', 'LineStyle', '--');
  xlabel('Actual Angle');
  ylabel('Estimated Angle');

figure(4);
  subplot(2,1,2);
  plot(x,squared_error);
  xlabel('Actual Angle');
  ylabel('Sqaured Error');
  title('Squared Error (Composite)');

figure(5); hold on;
    plot(x,squared_error, 'k');
    legend('(1,2)', '(2,3)', '(3,1)', 'Composite', 'location', 'North');
%% This script demos the simple phase difference angle-of-arrival (AoA)
%% calculation used by SeaBeaIII to estimate the direction of the sonar pinger.
%% This demo considers only the simple AoA estimation using two hydrophones.
%% Zero degrees is defined as directly ahead (normal to sensor plane).

%% Initialize Environment
clear all; close all;


%% Global Vars
N.samples = 512;
N.fft = 512;
SPEED_SOUND_WATER = 1482; % [m/s]

SIGNAL_FREQ = 26e3;
SAMPLING_FREQ = 104706; % [Hz]
WAVELENGTH = SPEED_SOUND_WATER / SIGNAL_FREQ;

SENSOR_SPACING = 0.02;

%% Generate Input Data
DISTANCE = 100; % [m]

%% File I/O
testDataFile = fopen('TestData2.H', 'w+');
fprintf(testDataFile, '#ifndef SEABEA_SONAR_TESTDATA_H\n#define SEABEA_SONAR_TESTDATA_H\n\n');



x = -90:1:90;
y = zeros(1,length(x));
for i=1:length(x)
    ANGLE = x(i); % [degrees]
    offset.h = DISTANCE * sin( ANGLE * pi / 180);
    offset.v = DISTANCE * cos( ANGLE * pi / 180);
    d1 = sqrt((offset.h + SENSOR_SPACING/2)^2 + offset.v^2);
    d2 = sqrt((offset.h - SENSOR_SPACING/2)^2 + offset.v^2);
    delta = d2 - d1;
    PHASE_DIFFERENCE = delta / (WAVELENGTH/2) * pi;

    observation_1 = sin(2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1);
    observation_2 = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - PHASE_DIFFERENCE);
    


    %% Plot Signal Input
    %figure; hold on;
    %plot(observation_1, 'r')
    %plot(observation_2, 'g')


    %% FFT
    data.fft1 = fft(observation_1, N.fft);
    data.fft2 = fft(observation_2, N.fft);

    [data.fft1_maxValue data.fft1_maxIndex] = max(data.fft1(1:N.fft/2));
    data.fft1_maxIndex;
    [data.fft2_maxValue data.fft2_maxIndex] = max(data.fft2(1:N.fft/2));

    data.phase1 = atan2(imag(data.fft1(data.fft1_maxIndex)), real(data.fft1(data.fft1_maxIndex))); %angle(data.fft1);
    data.phase2 = atan2(imag(data.fft2(data.fft2_maxIndex)), real(data.fft2(data.fft2_maxIndex))); %angle(data.fft2);

    phase_difference = data.phase2 - data.phase1;
    if phase_difference < -pi
        phase_difference = phase_difference + 2 * pi;
    end

    if phase_difference > pi
        phase_difference = phase_difference - 2 * pi;
    end

    y(i) = phase_difference * ((WAVELENGTH / 2) / SENSOR_SPACING) * 180 / pi / 2;
    
    if i == -20 + 90 + 1
        fprintf(testDataFile, 'double data1[512] = {');
        for j=1:N.samples-1
            fprintf(testDataFile, '%3.15f, ', observation_1(j));
        end
        fprintf(testDataFile, '%3.15f};\n', observation_1(N.samples));

        fprintf(testDataFile, 'double data2[512] = {');
        for j=1:N.samples-1
            fprintf(testDataFile, '%3.15f, ', observation_2(j));
        end
        fprintf(testDataFile, '%3.15f};\n', observation_2(N.samples));

        fprintf(testDataFile, 'double data3[512] = {');
        for j=1:N.samples-1
            fprintf(testDataFile, '0, ');
        end
        fprintf(testDataFile, '0};\n');


        fprintf(testDataFile, '\n#endif // SEABEA_SONAR_TESTDATA_H Defined\n\n');

        fclose(testDataFile);
        y(i)
        break;
    end
    
end

figure;
    plot(x,y)
    line([-90 90], [-90 90], 'Color', 'green', 'LineStyle', '--');
    xlabel('input');
    ylabel('output');


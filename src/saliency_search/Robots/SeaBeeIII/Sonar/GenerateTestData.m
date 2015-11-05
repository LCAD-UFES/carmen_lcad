function [ ] = GenerateTestData( angle )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% File I/O
testDataFile = fopen('TestData.H', 'w+');
fprintf(testDataFile, '#ifndef SEABEA_SONAR_TESTDATA_H\n#define SEABEA_SONAR_TESTDATA_H\n\n');


%% Global Vars
N.samples = 512;
N.fft = 512;
SPEED_SOUND_WATER = 1482; % [m/s]

SIGNAL_FREQ = 26e3;
SAMPLING_FREQ = 104706; % [Hz]
WAVELENGTH = SPEED_SOUND_WATER / SIGNAL_FREQ;

SENSOR_SPACING = 0.02;

%% Generate Input Data
DISTANCE = 2; % [m]

offset.h = DISTANCE * sin( angle * pi / 180);
offset.v = DISTANCE * cos( angle * pi / 180);
d1 = sqrt((offset.h + SENSOR_SPACING/2)^2 + offset.v^2);
d2 = sqrt((offset.h - SENSOR_SPACING/2)^2 + offset.v^2);
delta = d2 - d1;
PHASE_DIFFERENCE = delta / (WAVELENGTH/2) * pi;

data.input1 = sin(2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1);
data.input2 = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - PHASE_DIFFERENCE);

fprintf(testDataFile, 'double data1[512] = {');
for i=1:N.samples-1
    fprintf(testDataFile, '%3.15f, ', data.input1(i));
end
fprintf(testDataFile, '%3.15f};\n', data.input1(N.samples));

fprintf(testDataFile, 'double data2[512] = {');
for i=1:N.samples-1
    fprintf(testDataFile, '%3.15f, ', data.input2(i));
end
fprintf(testDataFile, '%3.15f};\n', data.input2(N.samples));

fprintf(testDataFile, 'double data3[512] = {');
for i=1:N.samples-1
    fprintf(testDataFile, '0, ');
end
fprintf(testDataFile, '0};\n');


fprintf(testDataFile, '\n#endif // SEABEA_SONAR_TESTDATA_H Defined\n\n');

fclose(testDataFile);
end


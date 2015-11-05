function [  ] = generate_observations( distance, angle )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global sensor1 sensor2 sensor3 WAVELENGTH SIGNAL_FREQ SAMPLING_FREQ N

sensor1.distance = sqrt((distance*cos(angle) - sensor1.x)^2 + ...
             (distance*sin(angle) - sensor1.y)^2);
sensor1.delta = mod(sensor1.distance, WAVELENGTH);
sensor1.phase = sensor1.delta / WAVELENGTH * 2 * pi;

sensor2.distance = sqrt((distance*cos(angle) - sensor2.x)^2 + ...
             (distance*sin(angle) - sensor2.y)^2);
sensor2.delta = mod(sensor2.distance, WAVELENGTH);
sensor2.phase = sensor2.delta / WAVELENGTH * 2 * pi;

sensor3.distance = sqrt((distance*cos(angle) - sensor3.x)^2 + ...
              (distance*sin(angle) - sensor3.y)^2);
sensor3.delta = mod(sensor3.distance, WAVELENGTH);
sensor3.phase = sensor3.delta / WAVELENGTH * 2 * pi;

%% Generate Sample Data
sensor1.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor1.phase);
sensor2.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor2.phase);
sensor3.observation = sin((2*pi*SIGNAL_FREQ/SAMPLING_FREQ * 1:1:N.samples+1) - sensor3.phase);

end


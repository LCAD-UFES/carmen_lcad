clear all; close all;

%% Define Globals
debug = true;
SPEED_SOUND_WATER = 1482; % [m/s]
SIGNAL_FREQ = 30e3;
SAMPLING_FREQ = 104706; % [Hz]
WAVELENGTH = SPEED_SOUND_WATER / SIGNAL_FREQ;
SENSOR_SPACING = 0.024; % [m] (between each possible paring)
signal_offset = 600;
firstpass_fft.N = 32;
firstpass_fft.expected_bin = firstpass_fft.N / SAMPLING_FREQ * SIGNAL_FREQ
firstpass_fft.expected_bin = round(firstpass_fft.expected_bin)+1


%% Load Sample Data
data = load('data_noconnect_times2_minus200_uncalib2.csv');




firstpass_fft.length = floor(length(data(600:length(data))) / firstpass_fft.N)
bin_mag_history      = zeros(firstpass_fft.length, 3);
bin_phase_history    = zeros(firstpass_fft.length, 3);

l = 10;
bin_l_mean_history   = zeros(firstpass_fft.length, 3);
bin_l_var_history    = zeros(firstpass_fft.length, 3);

for i=0:firstpass_fft.length-1
  fft_output.adc0 = fft(data(signal_offset + i*firstpass_fft.N+1:signal_offset + (i+1)*firstpass_fft.N,1), firstpass_fft.N);
  fft_output.adc1 = fft(data(signal_offset + i*firstpass_fft.N+1:signal_offset + (i+1)*firstpass_fft.N,2), firstpass_fft.N);
  fft_output.adc2 = fft(data(signal_offset + i*firstpass_fft.N+1:signal_offset + (i+1)*firstpass_fft.N,3), firstpass_fft.N);
  
  bin_mag_history(i+1,1) = imag(fft_output.adc0(firstpass_fft.expected_bin))^2 + real(fft_output.adc0(firstpass_fft.expected_bin)) ^2;
  bin_mag_history(i+1,2) = imag(fft_output.adc1(firstpass_fft.expected_bin))^2 + real(fft_output.adc1(firstpass_fft.expected_bin)) ^2;
  bin_mag_history(i+1,3) = imag(fft_output.adc2(firstpass_fft.expected_bin))^2 + real(fft_output.adc2(firstpass_fft.expected_bin)) ^2;
  
  bin_phase_history(i+1,1) = atan2(imag(fft_output.adc0(firstpass_fft.expected_bin)), real(fft_output.adc0(firstpass_fft.expected_bin)));
  bin_phase_history(i+1,2) = atan2(imag(fft_output.adc1(firstpass_fft.expected_bin)), real(fft_output.adc1(firstpass_fft.expected_bin)));
  bin_phase_history(i+1,3) = atan2(imag(fft_output.adc2(firstpass_fft.expected_bin)), real(fft_output.adc2(firstpass_fft.expected_bin)));
  
  if i > l
    bin_l_mean_history(i,1) = mean(bin_mag_history(i-l:i,1));
    bin_l_mean_history(i,2) = mean(bin_mag_history(i-l:i,2));
    bin_l_mean_history(i,3) = mean(bin_mag_history(i-l:i,3));
    
    bin_l_var_history(i,1) = var(bin_mag_history(i-l:i,1));
    bin_l_var_history(i,2) = var(bin_mag_history(i-l:i,2));
    bin_l_var_history(i,3) = var(bin_mag_history(i-l:i,3));
  end
end

ROI = 950:980;

%% Plot Raw Data Input
figure(1);
  subplot(3,1,1);
  plot(1:length(data(signal_offset:length(data),1)), data(signal_offset:length(data),1));
  title('ADC0 Data');
  subplot(3,1,2);
  plot(1:length(data(signal_offset:length(data),2)), data(signal_offset:length(data),2));
  title('ADC1 Data');
  subplot(3,1,3);
  plot(1:length(data(signal_offset:length(data),3)), data(signal_offset:length(data),3));
  title('ADC2 Data');

mag_plot = figure(2);
  subplot(3,1,1);
  title('ADC0 - Magnitude History');
  plot(bin_mag_history(ROI,1));
  subplot(3,1,2);
  title('ADC1 - Magnitude History');
  plot(bin_mag_history(ROI,2));
  subplot(3,1,3);
  title('ADC2 - Magnitude History');
  plot(bin_mag_history(ROI,3));


phase_plot = figure(3);
  subplot(3,1,1);
  title('2 -1  Phase History');
  plot(bin_phase_history(ROI,2) - bin_phase_history(ROI,1));
  subplot(3,1,2);
  title('3 - 2 Phase History');
  plot(bin_phase_history(ROI,3) - bin_phase_history(ROI,2));
  subplot(3,1,3);
  title('1 - 3 Phase History');
  plot(bin_phase_history(ROI,1) - bin_phase_history(ROI,3));


phase_plot = figure(4);
  subplot(3,1,1);
  title('ADC0 Mag Mean History');
  plot(bin_l_mean_history(ROI,1));
  subplot(3,1,2);
  title('ADC1 Mag Mean History');
  plot(bin_l_mean_history(ROI,2));
  subplot(3,1,3);
  title('ADC2 Mag Mean History');
  plot(bin_l_mean_history(ROI,3));

phase_plot = figure(5);
  subplot(3,1,1);
  title('ADC0 Mag Var History');
  plot(bin_l_var_history(ROI,1));
  subplot(3,1,2);
  title('ADC1 Mag Var History');
  plot(bin_l_var_history(ROI,2));
  subplot(3,1,3);
  title('ADC2 Mag Var History');
  plot(bin_l_var_history(ROI,3));

clear all; close all;

%% Define Globals
SPEED_SOUND_WATER = 1482; % [m/s]
SIGNAL_FREQ = 30e3;
SAMPLING_FREQ = 104706; % [Hz]
WAVELENGTH = SPEED_SOUND_WATER / SIGNAL_FREQ;
SENSOR_SPACING = 0.024; % [m] (between each possible paring)
signal_offset = 500;%600 + 115 * 32;
firstpass_fft.N = 32;
firstpass_fft.expected_bin = firstpass_fft.N / SAMPLING_FREQ * SIGNAL_FREQ
firstpass_fft.expected_bin = round(firstpass_fft.expected_bin)+1

min_signal_samples = 128;
MEAN_SCALE_FACTOR = 10;

%% Load Sample Data
%data = load('data_noconnect_times2.csv');
%data = load('data_noconnect_times2_minus200_uncalib1.csv');
data = load('data_noconnect_times2_minus200_uncalib2.csv');
%data = load('data_noconnect_times2_minus200_uncalib3.csv');
%data = load('data_noconnect_times2_minus200_uncalib4_90rot.csv');
%data = load('data_noconnect_times2_minus200_uncalib5_90rot.csv');


firstpass_fft.length = floor(length(data(signal_offset:length(data))) / firstpass_fft.N)
bin_mag              = zeros(1, 3);
bin_mag_history      = zeros(firstpass_fft.length, 3);
bin_phase_history    = zeros(firstpass_fft.length, 3);
bin_indicators       = zeros(firstpass_fft.length, 1);
idx_bin_min         = 0;
idx_bin_max         = 0;

l = 10;
bin_l_mean_history   = zeros(l, 3);
idx_bin_l_mean       = 1;

for i=0:firstpass_fft.length-2
  %% 'Real-time' Processing
  fft_output.adc0 = fft(data(signal_offset + i*firstpass_fft.N+1:signal_offset + (i+1)*firstpass_fft.N,1), firstpass_fft.N);
  fft_output.adc1 = fft(data(signal_offset + i*firstpass_fft.N+1:signal_offset + (i+1)*firstpass_fft.N,2), firstpass_fft.N);
  fft_output.adc2 = fft(data(signal_offset + i*firstpass_fft.N+1:signal_offset + (i+1)*firstpass_fft.N,3), firstpass_fft.N);
  
  %% Magnitude
  bin_mag_history(i+1,1) = imag(fft_output.adc0(firstpass_fft.expected_bin))^2 + real(fft_output.adc0(firstpass_fft.expected_bin)) ^2;
  bin_mag_history(i+1,2) = imag(fft_output.adc1(firstpass_fft.expected_bin))^2 + real(fft_output.adc1(firstpass_fft.expected_bin)) ^2;
  bin_mag_history(i+1,3) = imag(fft_output.adc2(firstpass_fft.expected_bin))^2 + real(fft_output.adc2(firstpass_fft.expected_bin)) ^2;
  
  bin_mag(1) = bin_mag_history(i+1,1);
  bin_mag(2) = bin_mag_history(i+1,2);
  bin_mag(3) = bin_mag_history(i+1,3);
  
  %% Phase
  bin_phase_history(i+1,1) = atan2(imag(fft_output.adc0(firstpass_fft.expected_bin)), real(fft_output.adc0(firstpass_fft.expected_bin)));
  bin_phase_history(i+1,2) = atan2(imag(fft_output.adc1(firstpass_fft.expected_bin)), real(fft_output.adc1(firstpass_fft.expected_bin)));
  bin_phase_history(i+1,3) = atan2(imag(fft_output.adc2(firstpass_fft.expected_bin)), real(fft_output.adc2(firstpass_fft.expected_bin)));
  
  %% Index Check
  if idx_bin_l_mean > l
    idx_bin_l_mean = 1;
  end
  
  %% Initialize Mean array
  if i < l + 1
    bin_l_mean_history(idx_bin_l_mean, 1) = bin_mag(1);
    bin_l_mean_history(idx_bin_l_mean, 2) = bin_mag(2);
    bin_l_mean_history(idx_bin_l_mean, 3) = bin_mag(3);
    idx_bin_l_mean = idx_bin_l_mean + 1;
    continue;
  end
  
  bin_l_mean = [mean(bin_l_mean_history(:, 1)) mean(bin_l_mean_history(:, 2)) mean(bin_l_mean_history(:, 3))];
  
  if bin_mag(1) > MEAN_SCALE_FACTOR * bin_l_mean(1) && bin_mag(2) > MEAN_SCALE_FACTOR * bin_l_mean(2) && bin_mag(3) > MEAN_SCALE_FACTOR * bin_l_mean(3)
    %i
    if idx_bin_min == 0
      idx_bin_min = i;
    end
    idx_bin_max = i+1;
    bin_indicators(i+1) = 1;
  else
    bin_l_mean_history(idx_bin_l_mean, 1) = bin_mag(1);
    bin_l_mean_history(idx_bin_l_mean, 2) = bin_mag(2);
    bin_l_mean_history(idx_bin_l_mean, 3) = bin_mag(3);
    idx_bin_l_mean = idx_bin_l_mean + 1;
    bin_l_mean = [mean(bin_l_mean_history(:, 1)) mean(bin_l_mean_history(:, 2)) mean(bin_l_mean_history(:, 3))];
  end
  
  if bin_indicators(i) == 0 && bin_indicators(i-1) == 1
    disp('Just had a pulse!');

    [idx_bin_min, idx_bin_max]    
    if idx_bin_max - idx_bin_min >= round(min_signal_samples / firstpass_fft.N) - 1
      figure();
      mag_data = abs(fft(data(signal_offset + idx_bin_min*firstpass_fft.N+1:signal_offset + (idx_bin_max+1)*firstpass_fft.N,1)));
      subplot(2,1,1)
      plot(data(signal_offset + idx_bin_min*firstpass_fft.N+1:signal_offset + (idx_bin_max+1)*firstpass_fft.N,1))
      subplot(2,1,2)
      plot(mag_data(2:length(mag_data)));
      sub_bin_expectation = round(length(mag_data) / SAMPLING_FREQ * SIGNAL_FREQ);
      line([sub_bin_expectation sub_bin_expectation], [-10 200]);
      drawnow;
    end
    
    idx_bin_min = 0;
    idx_bin_max = 0;
  end

end

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
  subplot(3,2,1);
  plot(bin_mag_history(:,1));
  title('ADC0 - Magnitude History');
  subplot(3,2,2);
  plot(bin_mag_history(:,1) .* bin_indicators);
  title('ADC0 - Magnitude History (Filtered)');
  subplot(3,2,3);
  plot(bin_mag_history(:,2));
  title('ADC1 - Magnitude History');
  subplot(3,2,4);
  plot(bin_mag_history(:,2) .* bin_indicators);
  title('ADC1 - Magnitude History (Filtered)');
  subplot(3,2,5);
  title('ADC2 - Magnitude History');
  plot(bin_mag_history(:,3));
  subplot(3,2,6);
  title('ADC2 - Magnitude History (Filtered)');
  plot(bin_mag_history(:,3) .* bin_indicators);


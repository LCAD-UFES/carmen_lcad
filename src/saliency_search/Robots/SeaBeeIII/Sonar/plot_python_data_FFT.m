clear all; close all; clc;

num_slaves = 3;
N_fft = 512;
Fs_total = 311403.000000;
Fs_slave = Fs_total / num_slaves;

data = load('data.dat');

[len dim] = size(data);

Y_1 = fft(data(:,1), N_fft) / len;
Y_2 = fft(data(:,2), N_fft) / len;
Y_3 = fft(data(:,3), N_fft) / len;

%% Calculat the argmax and value of each FFT
[val, index] = max(20*abs(Y_1(2:N_fft/2+1)));
F_1_max = Fs_slave / N_fft * index;
[val, index] = max(20*abs(Y_2(2:N_fft/2+1)));
F_2_max = Fs_slave / N_fft * index;
[val, index] = max(20*abs(Y_3(2:N_fft/2+1)));
F_3_max = Fs_slave / N_fft * index;

fprintf(1, '\nF_1_max: %5.5f kHz', F_1_max / 1000);
if F_1_max > 25000 && F_1_max < 35000
    fprintf(1, '  Pinger Peak detected on ADC 1!');
end

fprintf(1, '\nF_2_max: %5.5f kHz', F_2_max / 1000);
if F_2_max > 25000 && F_2_max < 35000
    fprintf(1, '  Pinger Peak detected on ADC 2!');
end

fprintf(1, '\nF_3_max: %5.5f kHz', F_3_max / 1000);
if F_3_max > 25000 && F_3_max < 35000
    fprintf(1, '  Pinger Peak detected on ADC 3!');
end

fprintf('\n');

%% Plot the FFT Results
figure; hold on;
subplot(2,2,1);
	plot(1:1:length(Y_1)/2, 20*abs(Y_1(2:N_fft/2+1)));
	title(sprintf('ADC 1 (max: %4.3f kHz)', F_1_max/1000));
	set(gca,'XTick',0:N_fft/8:N_fft/2)
	set(gca,'XTickLabel',{'0', sprintf('%4.1f',Fs_slave/N_fft/16), sprintf('%4.1f',Fs_slave/N_fft/8), sprintf('%4.1f',Fs_slave/N_fft/4), sprintf('%4.1f',Fs_slave/N_fft/2)})
	xlabel('Frequency [kHz]');
subplot(2,2,2);
	plot(1:1:length(Y_2)/2, 20*abs(Y_2(2:N_fft/2+1)));
	title(sprintf('ADC 2 (max: %4.3f kHz)', F_2_max/1000));
	set(gca,'XTick',0:N_fft/8:N_fft/2)
	set(gca,'XTickLabel',{'0', sprintf('%4.1f',Fs_slave/N_fft/16), sprintf('%4.1f',Fs_slave/N_fft/8), sprintf('%4.1f',Fs_slave/N_fft/4), sprintf('%4.1f',Fs_slave/N_fft/2)})
	xlabel('Frequency [kHz]');
subplot(2,2,3);
	plot(1:1:length(Y_3)/2, 20*abs(Y_3(2:N_fft/2+1)));
	title(sprintf('ADC 3 (max: %4.3f kHz)', F_3_max/1000));
	set(gca,'XTick',0:N_fft/8:N_fft/2)
	set(gca,'XTickLabel',{'0', sprintf('%4.1f',Fs_slave/N_fft/16), sprintf('%4.1f',Fs_slave/N_fft/8), sprintf('%4.1f',Fs_slave/N_fft/4), sprintf('%4.1f',Fs_slave/N_fft/2)})
	xlabel('Frequency [kHz]');



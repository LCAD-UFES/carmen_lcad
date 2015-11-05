clear all; close all; clc;

num_slaves = 3;
N_fft = 2048;
Fs_total = 374634.146341;
Fs_slave = Fs_total / num_slaves;

data1 = load('data.1.dat');
data2 = load('data.2.dat');
data3 = load('data.3.dat');

L_1 = numel(data1); Y_1 = fft(data1(:), N_fft) / L_1;
L_2 = numel(data1); Y_2 = fft(data2(:), N_fft) / L_2;
L_3 = numel(data1); Y_3 = fft(data3(:), N_fft) / L_3;

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



clear 
close all
% Open the file for reading
fid = fopen('data.txt', 'r');
controlfile = fopen('control.txt', 'r');

% fid = fopen('Copy_of_data_2.txt', 'r');
% controlfile = fopen('Copy_of_control.txt', 'r');
% fid = fopen('data.txt', 'r');

% Skip the header line
% % headerLine = fgetl(fid);

% Use textscan to read the data
data = textscan(fid, 't:%f fc:%f , %f psi1:%f , %f psi2:%f , %f psi3:%f , %f');
control_data = textscan(controlfile,'control : %f,a_filt_atk : %f,a_filt : %f,a_raw : %f');

% Close the file
fclose(fid);
fclose(controlfile);
% Convert the cell array to a numeric array
data = cell2mat(data);
control_data = cell2mat(control_data);

% Extract the columns
t = data(:, 1);
est = data(:, 2);
reel = data(:, 3);
% 
% fc = 10; % Cut off frequency
% fs = 30; % Sampling rate
% 
% [b,a] = butter(6,fc/(fs/2)); % Butterworth filter of order 6
% x = filter(b,a,control_data(:,3)); % Will be the filtered signal
% 
% % Compute the FFT of the signal
% fft_signal = fft(control_data(:,3));
% f = linspace(0,fs,length(control_data(:,3)));
% 
% 
% 
% control_filtred = filter(Hd,control_data(:,3));
% fft_signal_filted = fft(control_filtred);
% 
% % Plot the data
% figure(5)
% plot(f, abs(fft_signal));
% hold on
% plot(f, abs(fft_signal_filted));
% 
% % Plot the data
% figure(4)
% plot(t, control_data(:,2), 'b-');
% hold on
% plot(t, control_data(:,3), 'g');
% hold on
% plot(t, control_filtred, 'r');
% 
% 
% 

% % Plot the data
figure(5)
plot(t, control_data(:,3), 'b-');
hold on
plot(t, control_data(:,4), 'g');
grid on;

% % Plot the data
figure(3)
plot(t, control_data(:,1), 'b-');
xlabel('Time');
ylabel('Value');
title('Control');
grid on;


%% Results
figure(1)
subplot(3,1,1)
set(gcf,'color','w');
plot(t,data(:, 4),'r','LineWidth',1.5);
hold on
plot(t,data(:, 5),'b--','LineWidth',1.5);
grid on;
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
ylabel('$$\delta_i$$','Interpreter','LaTeX','Fontsize',12);
d1 = legend('$$\hat{\delta_i}$$','$$\delta_i$$');
d1.Interpreter = "latex";
d1.FontSize = 13;
d11 = title('Relative position between host and preceding vehicle and its estimation');
d11.Interpreter = "latex";
d11.FontSize = 10;



subplot(3,1,2)
set(gcf,'color','w');
plot(t,data(:, 6),'r','LineWidth',1.5);
hold on
plot(t,data(:, 7),'b--','LineWidth',1.5);
grid on;
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
ylabel('\boldmath$\dot{\delta}_i$','Interpreter','LaTeX','Fontsize',12);
d2 = legend('$\dot{\hat{\delta}_i}$','$$\boldmath{\dot{\delta}_i}$$');
d2.Interpreter = "latex";
d2.FontSize = 13;
d21 = title('Relative velocity between host and preceding vehicle and its estimation');
d21.Interpreter = "latex";
d21.FontSize = 10;



subplot(3,1,3)
set(gcf,'color','w');
plot(t,data(:, 8),'r','LineWidth',1.5);
hold on
plot(t,data(:, 9),'b--','LineWidth',1.5);
grid on;
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',12);
ylabel('\boldmath$\ddot{\delta}_i$','Interpreter','LaTeX','Fontsize',12);
d3 = legend('$$\ddot{\hat{\delta}_i}$$','$$\boldmath{\ddot{\delta}_i}$$');
d3.Interpreter = "latex";
d3.FontSize = 13;
d31 = title('Relative acceleration between host and preceding vehicle and its estimation');
d31.Interpreter = "latex";
d31.FontSize = 10;
ylim([-20 10]);

% ax = axes;
% set(ax,'units','normalized','position',[0.2,0.2,0.2,0.2])
% box(ax,'on')
% plot(psi_est_3,'r','LineWidth',1.5);
% hold on
% plot(psi_3,'b--','LineWidth',1.5);
% grid on;
% set(ax,'xlim',[0.0,0.25],'ylim',[-20,10])


%%
figure(2)
set(gcf,'color','w');
rgb1 = [0.4660 0.6740 0.1880];
rgb2 = [0 0.4470 0.7410];
subplot(2,1,1)

plot(t,reel,'Color',rgb1,'LineWidth',3);grid on
hold on
plot(t,est,'b--','LineWidth',4);grid on
% ylabel('$$f_{c}$$','Interpreter','LaTeX','Fontsize',12);
d4 = legend('$$f_{c}$$','${\hat{f}_c}$');
d4.Interpreter = "latex";
d4.FontSize = 13;
d41 = title('Cyber-attack and its estimation');
d41.Interpreter = "latex";
d41.FontSize = 10;
ylim([-30 40]);


subplot(2,1,2)
plot(t, control_data(:,3), 'b-','LineWidth',4);
hold on 
plot(t, control_data(:,2), 'r','LineWidth',4);
xlabel('{Time} ${[sec]}$','Interpreter','LaTeX','Fontsize',20);
title('Inter-Vehicle Comunication');
legend('a_{i-1}' ,'\mu_{i-1}');
grid on
% ylim([-13 10 ]);

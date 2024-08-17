clear; clc; close all;

%% declare
% sampling
Fs = 100;   % sampling freq [Hz]
dt = 1/Fs;  % sampling period [sec]

%% Load csv
% please change file name
file_name = 'Example_Sysid_Data.csv';
% 
csv_data = readmatrix(file_name);
t = csv_data(:,1)*dt;
vel = csv_data(:,2);
volt = csv_data(:,3);

%% system identification
IOdata = iddata(vel,volt,dt);
est_sys = tfest(IOdata,1,0);

%% disp
est_sys

%% plot
figure;
ax_vel = subplot(2,1,1);
ax_volt = subplot(2,1,2);
ax = [ax_vel ax_volt];

compare(ax_vel,IOdata,est_sys)
plot(ax_volt,t,volt)

grid(ax,'on')
xlabel(ax,'time (sec)')
title(ax_vel, 'vel')
title(ax_volt, 'volt')
ylabel(ax_vel,'rad/s')
ylabel(ax_volt,'volt')
% Process the data for the test results
clc
clear
close all
fid = fopen('data/TestResult.csv','r');
data = textscan(fid, '%f %f %f %f %f','Delimiter',',','HeaderLines',1);
fclose(fid);

time = data{1};
time = time - time(1);
time = time / 1000;
roll = data{2};
pitch = data{3};
yaw = data{4};
volt = data{5};

figure
plot(time,roll)
hold on
plot(time,pitch)
hold on
plot(time,yaw)
grid on
legend('Roll','Pitch','Yaw')
title("Motion of the Drone")
xlabel('Time [s]')
ylabel('Angle [deg]')

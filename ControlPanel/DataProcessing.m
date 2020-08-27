% Process the data for the test results
clc
clear
close all
fid = fopen('data/TestResult.csv','r');
data = textscan(fid, '%f %f %f %f %f %f %f %f %f','Delimiter',',','HeaderLines',1);
fclose(fid);

time = data{1};
time = time - time(1);
time = time / 1000;
% for i = 1:length(time)
%     if time(i) > 40
%         timeEnd = i-1;
%         break;
%     end
% end
roll = data{2};
pitch = data{3};
yaw = data{4};
pwm1 = data{5};
pwm2 = data{6};
pwm3 = data{7};
pwm4 = data{8};
volt = data{9};

% time = time(1:timeEnd);
% roll = roll(1:timeEnd);
% pitch = pitch(1:timeEnd);
% yaw = yaw(1:timeEnd);
% pwm1 = pwm1(1:timeEnd);
% pwm2 = pwm2(1:timeEnd);
% pwm3 = pwm3(1:timeEnd);
% pwm4 = pwm4(1:timeEnd);
% volt = volt(1:timeEnd);

figure
subplot(2,1,1)
plot(time,roll,'LineWidth',1.5)
hold on
plot(time,pitch,'LineWidth',1.5)
hold on
plot(time,yaw,'LineWidth',1.5)
grid on
legend('Roll','Pitch','Yaw')
title("Motion of the Drone")
xlabel('Time [s]')
ylabel('Angle [deg]')

subplot(2,1,2)
plot(time,pwm1)
hold on
plot(time,pwm2)
hold on
plot(time,pwm3)
hold on
plot(time,pwm4)
legend('Rotor 1','Rotor 2','Rotor 3','Rotor 4')
title("PWM vs Time")
xlabel('Time [s]')
ylabel('PWM')
grid on

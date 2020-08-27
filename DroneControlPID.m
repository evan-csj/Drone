clc
clear
close all

g = 9.8; % gravity
m = 1.1; % mass of the drone
b = 1.84E-7; % thrust coefficient
k = 3.24E-8; % air drag coefficient
L = 0.225; % distance from motor to center line
Ixx = 0.0118; % moment of inertia
Iyy = 0.0118;
Izz = 0.0225;
Tf = 0;
Ts = 0.01;

% phi theta psi phi_d theta_d psi_d
% U1 U2 U3
A = zeros(6,6);
B = zeros(6,3);
C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];

A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

B(4,1) = L*b/Ixx;
B(5,2) = L*b/Iyy;
B(6,3) = 1*k/Izz;

sys = ss(A,B,C,0);
sysd = c2d(sys,Ts);

Control = rank(ctrb(sys));
Observe = rank(obsv(sys));

drone = tf(sysd);

KpRoll = 6.5E6;
KiRoll = 3.8E6;
KdRoll = 2.8E6;

KpPitch = KpRoll;
KiPitch = KiRoll;
KdPitch = KdRoll;

KpYaw = 1.6E7;
KiYaw = 9.2E6;
KdYaw = 6.9E6;

pidRoll = pid(KpRoll,KiRoll,KdRoll,Tf,Ts);
pidPitch = pid(KpPitch,KiPitch,KdPitch,Tf,Ts);
pidYaw = pid(KpYaw,KiYaw,KdYaw,Tf,Ts);

tfRoll = feedback(pidRoll*drone(1,1),1);
tfPitch = feedback(pidPitch*drone(2,2),1);
tfYaw = feedback(pidYaw*drone(3,3),1);

[numX,denX] = tfdata(drone(1,1),'v');
[numY,denY] = tfdata(drone(2,2),'v');
[numZ,denZ] = tfdata(drone(3,3),'v');

t = 2.5;
roll0 = 10;
pitch0 = -10;
yaw0 = 5;
simOut = sim('Drone','SimulationMode','normal');
plot(simOut.RPY(:,1),rad2deg(simOut.RPY(:,2)))
hold on
plot(simOut.RPY(:,1),rad2deg(simOut.RPY(:,3)))
hold on
plot(simOut.RPY(:,1),rad2deg(simOut.RPY(:,4)))
grid on
title('Simulation Result of PID Control')
xlabel('Time [s]')
ylabel('Angle [deg]')
legend('Roll','Pitch','Yaw')
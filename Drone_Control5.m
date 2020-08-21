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
Ts = 0.007;

% phi theta psi phi_d theta_d psi_d
% U1 U2 U3
A = zeros(6,6);
B = zeros(6,3);
C = eye(6);

A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

B(4,1) = L*b/Ixx;
B(5,2) = L*b/Iyy;
B(6,3) = 1*k/Izz;

sys = ss(A,B,C,0);
% sysd = c2d(sys,Ts);

Control = rank(ctrb(sys));
Observe = rank(obsv(sys));

drone = tf(sys);

KpRoll = 435140;
KiRoll = 50400;
KdRoll = 939207;

KpPitch = KpRoll;
KiPitch = KiRoll;
KdPitch = KdRoll;

KpYaw = 1074342;
KiYaw = 125265;
KdYaw = 2303541;

pidRoll = pid(KpRoll,KiRoll,KdRoll);
pidPitch = pid(KpPitch,KiPitch,KdPitch);
pidYaw = pid(KpYaw,KiYaw,KdYaw);

tfRoll = feedback(pidRoll*drone(1,1),1);
tfPitch = feedback(pidPitch*drone(2,2),1);
tfYaw = feedback(pidYaw*drone(3,3),1);

[numX,denX] = tfdata(drone(1,1),'v');
[numY,denY] = tfdata(drone(2,2),'v');
[numZ,denZ] = tfdata(drone(3,3),'v');

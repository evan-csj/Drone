clc
clear
close all

g = 9.8; % gravity
m = 1.1; % mass of the drone
b = 1.98E-7; % thrust coefficient
k = 7.78E-6; % air drag coefficient
L = 0.160; % distance from motor to center line
Ixx = 0.0116; % moment of inertia
Iyy = 0.0225;
Izz = 0.0121;
Tf = 0;
Ts = 0.07;

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
sysd = c2d(sys,Ts);

Control = rank(ctrb(sys));
Observe = rank(obsv(sys));

drone = tf(sysd);

K = 935000;
Ti = 14.755;
Td = 1.8;

KpRoll = K;
KiRoll = K/Ti;
KdRoll = K*Td;

KpPitch = KpRoll*0.5;
KiPitch = KiRoll*0.5;
KdPitch = KdRoll*0.5;

KpYaw = 0;
KiYaw = 0;
KdYaw = 0;

pidRoll = pid(KpRoll,KiRoll,KdRoll,Tf,Ts);
pidPitch = pid(KpPitch,KiPitch,KdPitch,Tf,Ts);
pidYaw = pid(KpYaw,KiYaw,KdYaw,Tf,Ts);

tfRoll = feedback(pidRoll*drone(1,1),1);
tfPitch = feedback(pidPitch*drone(2,2),1);
tfYaw = feedback(pidYaw*drone(3,3),1);

t = 0:Ts:5;

figure
step(tfRoll,t)
hold on
step(tfPitch,t)
hold on
step(tfYaw)
grid on
legend('R','P','Y')

[numX,denX] = tfdata(drone(1,1),'v');
[numY,denY] = tfdata(drone(2,2),'v');
[numZ,denZ] = tfdata(drone(3,3),'v');

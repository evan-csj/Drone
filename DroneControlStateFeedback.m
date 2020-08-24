clc
clear
close all

g = 9.8; % gravity
m = 1; % mass of the drone
b = 1.98E-7; % thrust coefficient
k = 7.78E-6; % air drag coefficient
L = 160; % distance from motor to center line
Ixx = 0.0116; % moment of inertia
Iyy = 0.0225;
Izz = 0.0121;

% x y z phi theta psi u v w p q r
% U U1 U2 U3
A = zeros(12,12);
B = zeros(12,4);
C = eye(12);

A(1,7) = 1;
A(2,8) = 1;
A(3,9) = 1;
A(4,10) = 1;
A(5,11) = 1;
A(6,12) = 1;
A(7,5) = -g;
A(8,4) = g;

B(9,1) = 1/m;
B(10,2) = L/Ixx;
B(11,3) = L/Iyy;
B(12,4) = 1/Izz;

sys = ss(A,B,C,0);
Y = tf(sys);

Control = rank(ctrb(sys));
Observe = rank(obsv(sys));

polevec = [-0.97+0.22i; -0.97-0.22i;
           -0.90+0.43i; -0.90-0.43i;
           -0.78+0.62i; -0.78-0.62i;
           -0.62+0.78i; -0.62-0.78i;
           -0.43+0.90i; -0.43-0.90i;
           -0.22+0.97i; -0.22-0.97i;
           -1; -2; -3]*3;
       
Aaug = [A zeros(size(A,1),3); -C(1:3,:) zeros(3,3)];
Baug = [B; zeros(3,size(B,2))];
Kaug = place(Aaug,Baug,polevec);
K = Kaug(:,1:12);
Ka = Kaug(:,13:15);
A_new = Aaug - Baug*[K Ka];
B_new = [zeros(12,3);eye(3)];
C_new = [C(1:3,:) zeros(3,3)];

sys = ss(A_new,B_new,C_new,0);

xi = [0;0;0;0.5;0.5;0.5;0;0;0;0;0;0;0;0;0]; % initial state
dt = 0.05; % time step
t = 0:dt:50; % time
r = zeros(3,length(t)); % refenrece signal
r(1,:) = 0.3;

[~,~,x] = lsim(sys,r,t,xi);
xa = x(:,13:15);
x = x(:,1:12);
u = zeros(4,length(t));

for i = 1:length(t)
    u(:,i) = -Ka*xa(i,:).' - K*x(i,:).'; % input signal
end

% find the speed of each propeller from input u
u(1,:) = u(1,:) + m*g;
w1 = sqrt(-u(2,:)/b - u(3,:)/b + u(4,:)/k + u(1,:)/b)/2;
w2 = sqrt(-u(2,:)/b + u(3,:)/b - u(4,:)/k + u(1,:)/b)/2;
w3 = sqrt(+u(2,:)/b + u(3,:)/b + u(4,:)/k + u(1,:)/b)/2;
w4 = sqrt(+u(2,:)/b - u(3,:)/b - u(4,:)/k + u(1,:)/b)/2;

% nonlinear system
input = zeros(4,length(t));
state = zeros(12,length(t)+1);
state_a = zeros(3,length(t)+1);
stateDot = zeros(12,length(t)+1);
stateDot_a = zeros(3,length(t)+1);
state(:,1) = xi(1:12);
state_a(:,1) = xi(13:15);

for i = 1:length(t)
    % integrator state and dot
    stateDot_a(:,i) = r(:,i) - C(1:3,:)*state(:,i);
    state_a(:,i+1) = state_a(:,i) + stateDot_a(:,i)*dt;
    input(:,i) = -Ka*state_a(:,i) - K*state(:,i);
    
    % get x dot state
    stateDot(1,i) = cos(state(5,i))*cos(state(6,i))*state(7,i) + (-sin(state(6,i))*cos(state(4,i)) + sin(state(4,i))*sin(state(5,i))*cos(state(6,i)))*state(8,i) + (sin(state(6,i))*sin(state(4,i)) + cos(state(6,i))*cos(state(4,i))*sin(state(5,i)))*state(9,i);
    stateDot(2,i) = cos(state(5,i))*sin(state(6,i))*state(7,i) + (cos(state(4,i))*cos(state(6,i)) + sin(state(4,i))*sin(state(5,i))*sin(state(6,i)))*state(8,i) + (-sin(state(4,i))*cos(state(6,i)) + cos(state(4,i))*sin(state(5,i))*sin(state(6,i)))*state(9,i);
    stateDot(3,i) = -sin(state(5,i))*state(7,i) + sin(state(4,i))*cos(state(5,i))*state(8,i) + cos(state(4,i))*cos(state(5,i))*state(9,i);
    stateDot(4,i) = state(10,i) + sin(state(4,i))*tan(state(5,i))*state(11,i) + cos(state(4,i))*tan(state(5,i))*state(12,i);
    stateDot(5,i) = cos(state(4,i))*state(11,i) - sin(state(4,i))*state(12,i);
    stateDot(6,i) = sin(state(4,i))/cos(state(5,i))*state(11,i) + cos(state(4,i))/cos(state(5,i))*state(12,i);
    stateDot(7,i) = state(8,i)*state(12,i) - state(11,i)*state(9,i) - sin(state(5,i))*g;
    stateDot(8,i) = state(10,i)*state(9,i) - state(7,i)*state(12,i) + cos(state(5,i))*sin(state(4,i))*g;
    stateDot(9,i) = state(7,i)*state(11,i) - state(8,i)*state(10,i) + cos(state(4,i))*cos(state(5,i))*g*0 + input(1,i)/m;
    stateDot(10,i) = (Iyy - Izz)/Ixx*state(11,i)*state(12,i) + L*input(2,i)/Ixx;
    stateDot(11,i) = (Izz - Ixx)/Iyy*state(10,i)*state(12,i) + L*input(3,i)/Iyy;
    stateDot(12,i) = (Ixx - Iyy)/Izz*state(10,i)*state(11,i) + input(4,i)/Izz;
    
    % get x state
    state(1,i+1) = state(1,i) + stateDot(1,i)*dt;
    state(2,i+1) = state(2,i) + stateDot(2,i)*dt;
    state(3,i+1) = state(3,i) + stateDot(3,i)*dt;
    state(4,i+1) = state(4,i) + stateDot(4,i)*dt;
    state(5,i+1) = state(5,i) + stateDot(5,i)*dt;
    state(6,i+1) = state(6,i) + stateDot(6,i)*dt;
    state(7,i+1) = state(7,i) + stateDot(7,i)*dt;
    state(8,i+1) = state(8,i) + stateDot(8,i)*dt;
    state(9,i+1) = state(9,i) + stateDot(9,i)*dt;
    state(10,i+1) = state(10,i) + stateDot(10,i)*dt;
    state(11,i+1) = state(11,i) + stateDot(11,i)*dt;
    state(12,i+1) = state(12,i) + stateDot(12,i)*dt;
end

figure
subplot(3,4,1); plot(t,x(:,1)); title('x'); hold on; grid on
plot(t,state(1,1:end-1));
subplot(3,4,5); plot(t,x(:,2)); title('y'); hold on; grid on
plot(t,state(2,1:end-1));
subplot(3,4,9); plot(t,x(:,3)); title('z'); hold on; grid on
plot(t,state(3,1:end-1));
subplot(3,4,2); plot(t,x(:,4)); title('\phi'); hold on; grid on
plot(t,state(4,1:end-1));
subplot(3,4,6); plot(t,x(:,5)); title('\theta'); hold on; grid on
plot(t,state(5,1:end-1));
subplot(3,4,10); plot(t,x(:,6)); title('\psi'); hold on; grid on
plot(t,state(6,1:end-1));
subplot(3,4,3); plot(t,x(:,7)); title('u'); hold on; grid on
plot(t,state(7,1:end-1));
subplot(3,4,7); plot(t,x(:,8)); title('v'); hold on; grid on
plot(t,state(8,1:end-1));
subplot(3,4,11); plot(t,x(:,9)); title('w'); hold on; grid on
plot(t,state(9,1:end-1));
subplot(3,4,4); plot(t,x(:,10)); title('p'); hold on; grid on
plot(t,state(10,1:end-1));
subplot(3,4,8); plot(t,x(:,11)); title('q'); hold on; grid on
plot(t,state(11,1:end-1));
subplot(3,4,12); plot(t,x(:,12)); title('r'); hold on; grid on
plot(t,state(12,1:end-1));

figure
for i = 1:4
    subplot(4,1,i); plot(t,u(i,:)); grid on
end

figure
subplot(4,1,1); plot(t,w1); title('\omega1'); grid on
subplot(4,1,2); plot(t,w2); title('\omega2'); grid on
subplot(4,1,3); plot(t,w3); title('\omega3'); grid on
subplot(4,1,4); plot(t,w4); title('\omega4'); grid on


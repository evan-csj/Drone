clc
clear
close all

syms g m b k L Ixx Iyy Izz s

% phi theta psi phi_d theta_d psi_d
% U1 U2 U3
A = sym('A',[6,6]);
B = sym('B',[6,3]);
C = sym('C',[6,6]);
for i = 1:6
    for j = 1:6
        A(i,j) = 0;
        if(i == j) 
            C(i,j) = 1;
        else
            C(i,j) = 0;
        end
    end
    for j = 1:3
        B(i,j) = 0;
    end
end


A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;

B(4,1) = L*b/Ixx;
B(5,2) = L*b/Iyy;
B(6,3) = 1*k/Izz;

H = C*(s*C - A)^(-1)*B;


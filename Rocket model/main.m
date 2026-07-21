clear
clc
close all

P = rocketParameters();

disp("Rocket Parameters Loaded")

A = [0 1;
     0 0];

B = [0;
     1/P.Iyy];

Q = diag([100 10]);
R = 1;

K = lqr(A,B,Q,R);

disp("LQR Gain")
disp(K)
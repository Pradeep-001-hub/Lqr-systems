A = [0 1; 0 0];
B = [0; 1];
Q = [10 0; 0 1];
R = 1;
K = lqr(A,B,Q,R);
Acl = A - B*K;
t = 0:0.01:10;
x0 = [0.1; 0];
[x,t] = initial(Acl,x0,t);
plot(t,x(:,1))
xlabel('Time')
ylabel('Pitch Angle')
title('Rocket Stabilization using LQR')
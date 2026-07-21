function simulateRocket(P,K)

x0 = [
    deg2rad(10)
    0
];

tspan = [0 10];

[t,x] = ode45(@(t,x) closedLoopDynamics(t,x,P,K), tspan, x0);

% Compute control input
u = zeros(length(t),1);

for i=1:length(t)
    u(i)=lqrController(x(i,:)',K,P);
end

figure

subplot(3,1,1)
plot(t,rad2deg(x(:,1)),'LineWidth',2)
grid on
ylabel('Pitch (deg)')
title('Pitch Angle')

subplot(3,1,2)
plot(t,rad2deg(x(:,2)),'LineWidth',2)
grid on
ylabel('Rate (deg/s)')

subplot(3,1,3)
plot(t,rad2deg(u),'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('TVC (deg)')
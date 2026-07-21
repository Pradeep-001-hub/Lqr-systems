function simulateRocket(P)

% Initial state
x0 = [
    deg2rad(10);   % Initial pitch angle
    0              % Initial pitch rate
];

% Simulation time
tspan = [0 10];

% No control input
u = 0;

[t,x] = ode45(@(t,x) rocketDynamics(t,x,P,u), tspan, x0);

figure

subplot(2,1,1)
plot(t,rad2deg(x(:,1)),'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Pitch Angle (deg)')
title('Rocket Pitch Angle')

subplot(2,1,2)
plot(t,rad2deg(x(:,2)),'LineWidth',2)
grid on
xlabel('Time (s)')
ylabel('Pitch Rate (deg/s)')
title('Rocket Pitch Rate')
function P = rocketParameters()

% Physical Properties
P.mass = 5.0;              % kg
P.length = 1.5;            % m
P.diameter = 0.10;         % m

% Moment of inertia (pitch)
P.Iyy = 0.90;              % kg.m^2

% Gravity
P.g = 9.81;

% Aerodynamics
P.Cd = 0.45;
P.area = pi*(P.diameter/2)^2;
P.rho = 1.225;

% Thrust
P.maxThrust = 300;         % N

% TVC
P.maxGimbal = deg2rad(5);

end

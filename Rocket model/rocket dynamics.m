function dx = rocketDynamics(t, x, P, u)

% States
theta = x(1);      % Pitch angle (rad)
q     = x(2);      % Pitch rate (rad/s)

% Control input
delta = max(min(u, P.maxGimbal), -P.maxGimbal);

% Thrust-induced moment
Moment = P.maxThrust * P.length * sin(delta);

% Aerodynamic damping
Damping = -0.05 * q;

% Equations of motion
theta_dot = q;
q_dot = (Moment + Damping) / P.Iyy;

dx = [
    theta_dot;
    q_dot
];

end
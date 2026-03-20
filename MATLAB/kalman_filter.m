function [estimatedState, covariance] = kalman_filter(measurement, controlInput, dt)
    % Kalman Filter Implementation for Rocket Attitude Estimation
    % measurement - the measurement data (e.g., sensor readings)
    % controlInput - the control input (e.g., thrust, torque)
    % dt - time step

    % State vector: [angle; angular velocity]
    persistent state covariance
    if isempty(state)
        state = [0; 0]; % Initial state
        covariance = eye(2); % Initial covariance
    end
    
    % Process noise covariance
    Q = [1e-4, 0; 0, 1e-4]; 
    % Measurement noise covariance
    R = 1e-2; 
    
    % State transition model
    A = [1, dt; 0, 1]; 
    % Control input model
    B = [0.5 * dt^2; dt]; 
    % Measurement model
    H = [1, 0];

    % Prediction step
    state = A * state + B * controlInput; 
    covariance = A * covariance * A' + Q; 

    % Update step
    innovation = measurement - (H * state);
    innovationCovariance = H * covariance * H' + R; 
    kalmanGain = covariance * H' / innovationCovariance;

    state = state + kalmanGain * innovation; 
    covariance = (eye(size(kalmanGain, 1)) - kalmanGain * H) * covariance;

    estimatedState = state;
end

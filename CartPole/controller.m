function u = controller(X, params, X_desired)
    % PID controller constants
    Kp_x = 60;   % Proportional gain for position
    Ki_x = 0.02;     % Integral gain for position
    Kd_x = 175;    % Derivative gain for position
    Kp_theta = 1.3;  % Proportional gain for angle
    Ki_theta = 0.027;     % Integral gain for angle
    Kd_theta = 1.75;    % Derivative gain for angle
    
    persistent integral_error_x integral_error_theta
    
    % Initialize persistent variables
    if isempty(integral_error_x)
        integral_error_x = 0;
    end
    if isempty(integral_error_theta)
        integral_error_theta = 0;
    end
    
    % Extract states
    x = X(1);          % X-position of the Cart
    x_dot = X(2);      % Speed of Cart
    theta = X(3);      % Angular Position of the Pole
    theta_dot = X(4);  % Angular Speed of the Pole
    
    % Extract desired states
    x_desired = X_desired(1);          % Final Desired X-position of the Cart
    x_dot_desired = X_desired(2);      % Final Desired Velocity of the Cart
    theta_desired = X_desired(3);      % Final Desired Angular Position of the Pole
    theta_dot_desired = X_desired(4);  % Final Desired Angular Speed of the Pole
    
    % Calculate errors
    error_x = x_desired - x;
    error_x_dot = x_dot_desired - x_dot;
    error_theta = theta_desired - theta;
    error_theta_dot = theta_dot_desired - theta_dot;
    
    % Update integral errors
    integral_error_x = integral_error_x + error_x;
    integral_error_theta = integral_error_theta + error_theta;
    
    % Calculate control inputs
    u_x = Kp_x * error_x + Ki_x * integral_error_x + Kd_x * error_x_dot;
    u_theta = Kp_theta * error_theta + Ki_theta * integral_error_theta + Kd_theta * error_theta_dot;
    
    % Combine control inputs
    u = u_x + u_theta;
    
    % Ensure that control input is within limits
    u_max = params.u_max;  % Maximum allowable 
    u = min(max(u, -u_max), u_max);
end
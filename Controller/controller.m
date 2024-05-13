function [ F, M ] = controller(~, state, des_state, ~)
    %CONTROLLER  Controller for the planar quadrotor
    %
    %   state: The current state of the robot with the following fields:
    %   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
    %   state.omega = [phi_dot]
    %
    %   des_state: The desired states are:
    %   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
    %   [y_ddot; z_ddot]
    %
    %   params: robot parameters
    
    %   Using these current and desired states, you have to compute the desired
    %   controls
    
    persistent prev_error_integral 
    if isempty(prev_error_integral)
        prev_error_integral = [0; 0];
    end
    % Controller gains for position and attitude
    kp_pos = 20;
    kd_pos = 5;
    ki_pos = 0.1;
    kp_att = 0.001;
    kd_att = 0;
    
    % Time step
    dt = 0.001;
    
    % Current state
    pos_curr = state.pos;
    vel_curr = state.vel;
    rot_curr = state.rot;
    rot_curr = atan2(sin(rot_curr), cos(rot_curr));
    omega_curr = state.omega;
    
    % Desired state
    pos_des = des_state.pos;
    vel_des = des_state.vel;
    
    % Compute position error terms
    pos_error = pos_des - pos_curr;
    vel_error = vel_des - vel_curr;
    
    % Update integral error using trapezoidal integration
    prev_error_integral = prev_error_integral + pos_error * dt;
    
    % Calculate distance and angle of destination vector, and del_distance
    % and del_integral 
    distance = sqrt((pos_error(2))^2 + (pos_error(1))^2);
    angle = atan2(pos_error(2), pos_error(1));
    del_distance = (2 * pos_error(2) * vel_error(2) + 2 * pos_error(1) * vel_error(1));
    del_integral_distance = sqrt((prev_error_integral(2))^2 + (prev_error_integral(1))^2);
    
    % Compute control inputs for position
    F =  kp_pos * distance + kd_pos * del_distance + ki_pos * del_integral_distance; 
    
    % Compute attitude error (difference between desired and current orientation)
    att_error = angle - rot_curr;
    
    % Ensure att_error is within [-pi, pi] range
    att_error = atan2(sin(att_error), cos(att_error));
    
    % Compute control inputs for attitude (orientation)
    M = -kp_att * att_error +- kd_att * omega_curr(1); 
    
    % Update previous error for next iteration
    % prev_error = pos_error;
    
    end
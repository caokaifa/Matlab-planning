classdef param

    properties(Access=public)
        max_speed = 1.0; %[m/s]
        min_speed = -0.5;  % [m/s]
        max_yaw_rate = 40.0 * pi / 180.0;  % [rad/s]
        max_accel = 0.2;  % [m/ss]
        max_delta_yaw_rate = 40.0 * pi / 180.0;  % [rad/ss]
        v_resolution = 0.01;  % [m/s]
        yaw_rate_resolution = 0.1 * pi / 180.0;  % [rad/s]
        dt = 0.1;  % [s] Time tick for motion prediction
        predict_time = 3.0;  % [s]
        to_goal_cost_gain = 0.15;
        speed_cost_gain = 1.0;
        obstacle_cost_gain = 1.0;
        robot_type = 0;
        % if robot_type == RobotType.circle result=0
        % Also used to check if goal is reached in both types
         robot_radius = 1.0;  %[m] for collision check
         % if robot_type == RobotType.rectangle result=1
         robot_width = 0.5;  % [m] for collision check
         robot_length = 1.2;  % [m] for collision check
        
    end
end
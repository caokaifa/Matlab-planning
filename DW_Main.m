% main algorithm for Frenet path planning 
%Editor:Robert  Time:2020.8.16
clc
clear 
close all
disp('Starting')
  show_animation=true;
  if show_animation
      figure
  end
% initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
x=[0,0,pi/8,0, 0];
%   goal position [x(m), y(m)]
goal = [10,10];
% obstacles [x(m) y(m), ....]
ob=[-1, -1 ;
     0,  2 ;
     4,  2 ;
     5,  4 ;
     5,  5 ;
     5,  6 ;
     5,  9 ;
     8,  9 ;
     7,  9 ;
     8,  10 ;
     12, 13 ;
     12, 12 ;
     15, 15;
     13, 13] ;
%  input [forward speed, yaw_rate]
par=param;

trajectory=[];
trajectory=[trajectory x];
writerObj=VideoWriter('Dynamic Window Approach.avi'); %// 定义一个视频文件用来存动
open(writerObj); %// 打开该视频文件
while true
     [u,predicted_trajectory]= dwa_control(x, par, goal, ob);
      x = motion(x, u, par.dt);  % simulate robot
      trajectory=[trajectory x];  % store state history
    
     if(show_animation)
            cla()
            % for stopping simulation with the esc key.
            plot(predicted_trajectory(:, 1), predicted_trajectory(:, 2), "-g")
            hold on
            plot(x(1), x(2), "xr")
            plot(goal(1), goal(2), "xb")
            plot(ob(:,1), ob(:,2), "ok")
            axis("equal")
            frame = getframe; %// 把图像存入视频文件中
            frame.cdata = imresize(frame.cdata, [653 514]); %重新定义帧大小
            writeVideo(writerObj,frame); %// 将帧写入视频
     end
     
  % check reaching goal
    dist_to_goal = hypot(x(1) - goal(1), x(2)- goal(2));
    if dist_to_goal <= par.robot_radius
        print("Goal!!")
        close(writerObj); %// 关闭视频文件句柄  
        break
    end
    print("Done")
    if (show_animation)
        plot(trajectory(:, 1), trajectory(:, 2), "-r");  
    end    
end
    


   
function [u,predicted_trajectory]=dwa_control(x, par, goal, ob)
  
   % Dynamic Window Approach control
  
    dw = calc_dynamic_window(x, par);

    [u,predicted_trajectory] = calc_control_and_trajectory(x, dw, par, goal, ob);
    
end

function dw=calc_dynamic_window(x, par)
    
%     calculation dynamic window based on current state x
%      Dynamic window from robot specification

    Vs = [par.min_speed, par.max_speed,-par.max_yaw_rate, par.max_yaw_rate];

%      Dynamic window from motion model
    Vd = [x(4) - par.max_accel * par.dt,x(4) + par.max_accel * par.dt,x(5) - par.max_delta_yaw_rate * par.dt,x(5) + par.max_delta_yaw_rate * par .dt];

%       [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs(1), Vd(1)), min(Vs(2), Vd(2)),max(Vs(3), Vd(3)), min(Vs(4), Vd(4))];

end


function [best_u,best_trajectory]=calc_control_and_trajectory(x, dw, config, goal, ob)

%     calculation final input with dynamic window

    x_init = x;
    min_cost = inf;
    best_u = [0.0, 0.0];
    best_trajectory = NaN;

%      evaluate all trajectory with sampled input in dynamic window
    for v = dw(1):config.v_resolution:dw(2)
        for y = dw(3):config.yaw_rate_resolution:dw(4)

            trajectory = predict_trajectory(x_init, v, y, config);

%              calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal);
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory(end, 4));
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config);
% 
            final_cost = to_goal_cost + speed_cost + ob_cost;
% 
%              search minimum trajectory
            if min_cost >= final_cost
                min_cost = final_cost;
                best_u = [v, y];
                best_trajectory = trajectory;

            end       
               
        end
    end
end
   
function trajectory= predict_trajectory(x_init, v, y, config)
  
%     predict trajectory with an input
   

    x = x_init;
    trajectory = x;
    time = 0;
    while time <= config.predict_time
        x = motion(x, [v, y], config.dt);
        trajectory = [trajectory ;x];
        time =time+ config.dt;

    end
end

function x= motion(x, u, dt)
  
%     motion model
  
    x(3) =x(3)+ u(2) * dt;
    x(1) =x(1)+ u(1) * cos(x(3)) * dt;
    x(2) =x(2)+ u(1) * sin(x(3)) * dt;
    x(4) = u(1);
    x(5) = u(2);

end
 
function cost=calc_to_goal_cost(trajectory, goal)

  %      calc to goal cost with angle difference
 

    dx = goal(1) - trajectory(end,1);
    dy = goal(2) - trajectory(end,2);
    error_angle = atan2(dy, dx);
    cost_angle = error_angle - trajectory(end, 3);
    cost = abs(atan2(sin(cost_angle), cos(cost_angle)));

end

function cost= calc_obstacle_cost(trajectory, ob, config)

%         calc obstacle cost inf: collision

    ox = ob(:, 1);
    oy = ob(:, 2);
    cost=0;
    r1=[];
    for i=1:1:length(ox)
        dx = trajectory(:, 1) - ox(i);
        dy = trajectory(:,2 ) - oy(i);
        r = hypot(dx, dy);
        r1=[r1 r];
        if config.robot_type == 0
            if any(r <= config.robot_radius)
                return 
            end
        end
    end

   min_r = min(r1(:));
   cost= 1.0 / min_r;  % OK
 
end
  




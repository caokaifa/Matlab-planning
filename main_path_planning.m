% main algorithm for Frenet path planning 
clc
clear 
close all
set(0,'DefaultLineLineWidth',1);
disp('Optimal Frenet Path Planning')




%% 2.set up the trajectory
% 2.1 set up the track
w_x=[0.0, 10.0, 20.5, 30.0, 40.5, 50.0, 60.0];
w_y=[0.0, -4.0, 1.0, 6.5, 9.0, 10.0, 6.0];

% w_x=[0.0 10.0 20.5 35.0 70.5]
% w_y=[0 -6 5 6.5 0]

figure(1)
plot(w_x,w_y)
% 2.2 set up the obstacle
ob=[20.0, 10.0 ;
      30.0, 6.0 ;
      30.0, 5.0 ;
      35.0, 7.0 ;
      50.0, 12.0 ] ;
  hold on
  plot(ob(:,1),ob(:,2),'*r')
 % 2.3 create a reference track
  ds=0.1;    %discrete step size 
  GenerateTargetCourse = @(wx, wy) calcSplineCourse(wx, wy, ds);
  [RefX, RefY, RefYaw, RefCurvature, runningLength, referencePath]=...
      GenerateTargetCourse(w_x, w_y);
   hold on
  plot(RefX(1,:),RefY(1,:),'*r')
  
 % [rx, ry, ryaw, rk, s, oSpline]=calcSplineCourse(x,y,ds);
  
 % Initial state
  s0_d=10.0 / 3.6;          % current speed [m/s]
  d0 = 2.0;                      % current lateral position [m]
  d0_d=0;                       % current lateral speed [m/s]
  d0_dd = 0;                    % current lateral acceleration [m/s^2]
  s0= 0;                          % current course position[m]
  
  area=20;                      % animation area length[m]
  
  objFrenetPlanner = OptimalFrenetPlanner();
  show_animation=true;
  if show_animation
      figure
  end
  writerObj=VideoWriter('test.avi'); %// 定义一个视频文件用来存动画
  open(writerObj); %// 打开该视频文件
  %start simulation 
  T=500;
  for t = 1:T 
        t;
        trajectory = objFrenetPlanner.FrenetOptimalPlanning (...
            referencePath, s0, s0_d, d0, d0_d, d0_dd, ob);
        
        %store the updated state of the planned trajectorut as initial
        %state of next iteration for the new trajectory
        s0 = trajectory.s(2);
        s0_d= trajectory.ds(2);
        d0 = trajectory.d(2);
        d0_d = trajectory.dd(2);
        d0_dd=trajectory.ddd(2);
        
     

  if(show_animation)
      cla;
      plot(RefX,RefY);
      hold on
      axis equal
      plot(ob(:,1),ob(:,2),'xk')
      plot(trajectory.x(1:end),trajectory.y(1:end), '-ob');
%       plot(trajectory.x(1), trajectory.y(1), 'vc');
      grid on 
      drawnow
      if(sqrt((trajectory.x(1)-RefX(end))^2+(trajectory.y(1)-RefY(end))^2)<2.0)
          print("complete goal")
          close(writerObj); %// 关闭视频文件句柄   
          break
      end
      
      
      frame = getframe; %// 把图像存入视频文件中
      frame.cdata = imresize(frame.cdata, [653 514]); %重新定义帧大小
      writeVideo(writerObj,frame); %// 将帧写入视频
      
  end
  end
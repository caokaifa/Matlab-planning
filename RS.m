%%editor:Robert
%%Time:2010.8.23

% path = FindRSPath(5,1,pi);





Vehicle.WB = 3;  % [m] wheel base: rear to front steer
Vehicle.W = 2; % [m] width of vehicle
Vehicle.LF = 3.3; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle 
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius



%%起始点
start_x = 3.0;  %[m]
start_y = 4.0 ; % [m]
start_yaw = deg2rad(0.0);  % [rad]


%%终点
end_x = 8.0 ; % [m]
end_y = 0.0 ; % [m]
end_yaw =deg2rad(60.0);  %[rad]

%%转换坐标
[x,y,ang]=calc_paths(start_x,start_y,start_yaw,end_x,end_y,end_yaw);

path = FindRSPath(x,y,ang,Vehicle);
PlotPath(path,Vehicle,start_x,start_y,start_yaw,end_x,end_y,end_yaw);


function [x,y,ang]=calc_paths(sx, sy, syaw, gx, gy, gyaw)
q0 = [sx, sy, syaw];
q1 = [gx, gy, gyaw];
dx = q1(1) - q0(1);
dy = q1(2) - q0(2);
dth = q1(3) - q0(3);
c = cos(q0(3));
s = sin(q0(3));
x = (c * dx + s * dy);
y = (-s * dx + c * dy);
ang=dth;
end


function path = FindRSPath(x,y,phi,veh)
    rmin = veh.MIN_CIRCLE; %minimum turning radius
    x = x/rmin;
    y = y/rmin;
    % 遍历5种方法到达目标点，然后选取路径最短的一条
    [isok1,path1] = CSC(x,y,phi);
    [isok2,path2] = CCC(x,y,phi);
    [isok3,path3] = CCCC(x,y,phi);
    [isok4,path4] = CCSC(x,y,phi);
    [isok5,path5] = CCSCC(x,y,phi);
    isoks = [isok1, isok2, isok3, isok4, isok5];
    paths = {path1, path2, path3, path4, path5};
    Lmin = inf;
    % 找出5条路径最短的曲线
    for i = 1:5
        if isoks(i) == true
            elem = paths{i};
            if Lmin > elem.totalLength
                Lmin = elem.totalLength;
                path = elem;
            end
        end
    end
%     PlotPath(path,veh);
end

% 控制角度x取值范围是[-pi,pi]
function v = mod2pi(x)
    v = rem(x,2*pi);
    if v < -pi
        v = v+2*pi;
    elseif v > pi
        v = v-2*pi;
    end
end

% formula 8.6
function [tau,omega] = tauOmega(u,v,xi,eta,phi)
    delta = mod2pi(u-v);
    A = sin(u)-sin(delta);
    B = cos(u)-cos(delta)-1;
    t1 = atan2(eta*A-xi*B,xi*A+eta*B);
    t2 = 2*(cos(delta)-2*cos(v)-2*cos(u))+3;
    if t2 < 0
        tau = mod2pi(t1+pi);
    else
        tau = mod2pi(t1);
    end
    omega = mod2pi(tau-u+v-phi);
end

% formula 8.1
function [isok,t,u,v] = LpSpLp(x,y,phi)
    [t,u] = cart2pol(x-sin(phi),y-1+cos(phi)); % 将笛卡尔坐标转换为极坐标,返回theta和rho,论文返回的是[u,t],是因为cart2pol函数返回的值的顺序不同导致与原文不同，变量代表的含义还是一样，t代表弧度，u代表直行的距离
    if t >= 0 % 必须是左转,t>=0代表左转
        v = mod2pi(phi-t);
        if v >= 0 % 符号代表前进和后退
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.2
function [isok,t,u,v] = LpSpRp(x,y,phi)
    [t1,u1] = cart2pol(x+sin(phi),y-1-cos(phi));
    if u1^2 >= 4
        u = sqrt(u1^2-4);
        theta = atan2(2,u);
        t = mod2pi(t1+theta);
        v = mod2pi(t-phi);
        if t >= 0 && v >= 0 % 符号代表前进和后退
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

function [isok,path] = CSC(x,y,phi)
    Lmin = inf;
    type = repmat('N',[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpSpLp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(15,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(15,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(16,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpLp(-x,-y,phi); % timeflp + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(16,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(13,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(13,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(x,-y,-phi); % reflect 
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(14,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpSpRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(14,:),-t,-u,-v,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

% formula 8.3/8.4
function [isok,t,u,v] = LpRmL(x,y,phi)
    xi = x-sin(phi);
    eta = y-1+cos(phi);
    [theta,u1] = cart2pol(xi,eta);
    if u1 <= 4
        u = -2*asin(u1/4);
        t = mod2pi(theta+u/2+pi);
        v = mod2pi(phi-t+u);
        if t >= 0 && u <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

function [isok,path] = CCC(x,y,phi)
    Lmin = inf;
    type = repmat('N',[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmL(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(1,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(1,:),-t,-u,-v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(2,:),t,u,v,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(2,:),-t,-u,-v,0,0);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [isok,t,u,v] = LpRmL(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(1,:),v,u,t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(1,:),-v,-u,-t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(2,:),v,u,t,0,0);
        end
    end
    [isok,t,u,v] = LpRmL(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(2,:),-v,-u,-t,0,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

% formula 8.7,tauOmega() is formula 8.6
function [isok,t,u,v] = LpRupLumRm(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    rho = (2+sqrt(xi^2+eta^2))/4;
    if rho <= 1
        u = acos(rho);
        [t,v] = tauOmega(u,-u,xi,eta,phi);
        if t >= 0 && v <= 0 % 符号代表前进和后退
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.8
function [isok,t,u,v] = LpRumLumRp(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    rho = (20-xi^2-eta^2)/16;
    if rho >= 0 && rho <= 1
        u = -acos(rho);
        if u >= pi/2
            [t,v] = tauOmega(u,u,xi,eta,phi);
            if t >=0 && v >=0
                isok = true;
                return
            end
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

function [isok,path] = CCCC(x,y,phi)
    Lmin = inf;
    type = repmat('N',[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRupLumRm(x,y,phi);
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(3,:),t,u,-u,v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(3,:),-t,-u,u,-v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(4,:),t,u,-u,v,0);
        end
    end
    [isok,t,u,v] = LpRupLumRm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(4,:),-t,-u,u,-v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(x,y,phi);
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(3,:),t,u,u,v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(3,:),-t,-u,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(x,-y,-phi); % reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(4,:),t,u,u,v,0);
        end
    end
    [isok,t,u,v] = LpRumLumRp(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+2*abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(4,:),-t,-u,-u,-v,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

% formula 8.9
function [isok,t,u,v] = LpRmSmLm(x,y,phi)
    xi = x-sin(phi);
    eta = y-1+cos(phi);
    [theta,rho] = cart2pol(xi,eta);
    if rho >= 2
        r = sqrt(rho^2-4);
        u = 2-r;
        t = mod2pi(theta+atan2(r,-2));
        v = mod2pi(phi-pi/2-t);
        if t >= 0 && u <= 0 && v <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

% formula 8.10
function [isok,t,u,v] = LpRmSmRm(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    [theta,rho] = cart2pol(-eta,xi);
    if rho >= 2
        t = theta;
        u = 2-rho;
        v = mod2pi(t+pi/2-phi);
        if t >= 0 && u <= 0 && v <= 0
            isok = true;
            return
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

function [isok,path] = CCSC(x,y,phi)
    Lmin = inf;
    type = repmat('N',[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmSmLm(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(5,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(5,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(6,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(6,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(9,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-x,y,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(9,:),-t,pi/2,-u,-v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(x,-y,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(10,:),t,-pi/2,u,v,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-x,-y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(10,:),-t,pi/2,-u,-v,0);
        end
    end
    % backwards
    xb = x*cos(phi)+y*sin(phi);
    yb = x*sin(phi)-y*cos(phi);
    [isok,t,u,v] = LpRmSmLm(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(7,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(7,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(8,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmLm(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(8,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(xb,yb,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(11,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-xb,yb,-phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(11,:),-v,-u,pi/2,-t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(xb,-yb,-phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(12,:),v,u,-pi/2,t,0);
        end
    end
    [isok,t,u,v] = LpRmSmRm(-xb,-yb,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(12,:),-v,-u,pi/2,-t,0);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

% formula 8.11
function [isok,t,u,v] = LpRmSLmRp(x,y,phi)
    xi = x+sin(phi);
    eta = y-1-cos(phi);
    [~,rho] = cart2pol(xi,eta);
    if rho >= 2
        u = 4-sqrt(rho^2-4);
        if u <= 0
            t = mod2pi(atan2((4-u)*xi-2*eta,-2*xi+(u-4)*eta));
            v = mod2pi(t-phi);
            if t >= 0 && v >= 0
                isok = true;
                return
            end
        end
    end
    isok = false;
    t = 0;
    u = 0;
    v = 0;
end

function [isok,path] = CCSCC(x,y,phi)
    Lmin = inf;
    type = repmat('N',[1,5]);
    path = RSPath(type,0,0,0,0,0);
    [isok,t,u,v] = LpRmSLmRp(x,y,phi);
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(17,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(17,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(18,:),t,-pi/2,u,-pi/2,v);
        end
    end
    [isok,t,u,v] = LpRmSLmRp(x,y,phi); % timeflip + reflect
    if isok
        L = abs(t)+abs(u)+abs(v);
        if Lmin > L
            Lmin = L;
            path = RSPath(RSPath.Types(18,:),-t,pi/2,-u,pi/2,-v);
        end
    end
    if Lmin == inf
        isok = false;
    else
        isok = true;
    end
end

function PlotPath(path,veh,start_x,start_y,start_yaw,end_x,end_y,end_yaw)
    rmin = veh.MIN_CIRCLE;
    type = path.type;
    x = [];
    y = [];
    angle=[];
    seg = [path.t,path.u,path.v,path.w,path.x];
    pvec = [0,0,0];
    for i = 1:5 

        if type(i) == 'S'
            theta = pvec(3);
            dl = rmin*seg(i);
            t = theta+linspace(0,0);
            dvec = [dl*cos(theta), dl*sin(theta), 0];
            dx = pvec(1)+linspace(0,dvec(1));
            dy = pvec(2)+linspace(0,dvec(2));
%             x = [x,dx];
%             y = [y,dy];
%             angle=[angle,t];
            pvec = pvec+dvec;
           
        elseif type(i) == 'L'
            theta = pvec(3);
            dtheta = seg(i);
            cenx = pvec(1)-rmin*sin(theta);
            ceny = pvec(2)+rmin*cos(theta);
            t = theta-pi/2+linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
%             x = [x,dx];
%             y = [y,dy];
%             angle=[angle,t];
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl = dtheta;
        elseif type(i) == 'R'
            theta = pvec(3);
            dtheta = -seg(i);
            cenx = pvec(1)+rmin*sin(theta);
            ceny = pvec(2)-rmin*cos(theta);
            t = theta+pi/2+linspace(0,dtheta);
            dx = cenx+rmin*cos(t);
            dy = ceny+rmin*sin(t);
%             x = [x,dx];
%             y = [y,dy];
%             angle=[angle,t];
            theta = theta+dtheta;
            pvec = [dx(end),dy(end),theta];
            dl = -dtheta;
        else
            % do nothing
        end 
       if dl > 0 && (type(i) == 'R' || type(i) == 'L' || type(i) == 'S')
               % convert global coordinate
           ddx=[];
           ddy=[];
           ddyaw=[];
            for i=1:length(dx)
               dx_1= cos(-start_yaw) * dx(i) + sin(-start_yaw)* dy(i) + start_x;
               ddx=[ddx dx_1];
               dy_1= -sin(-start_yaw) * dx(i) + cos(-start_yaw)* dy(i) + start_y;
               ddy=[ddy dy_1];
               dyaw_1=pi_2_pi(t(i)+ start_yaw);
               ddyaw = [ddyaw dyaw_1];

            end
          x = [x,ddx];
          y = [y,ddy];
          angle=[angle,ddyaw];
          plot(ddx,ddy,'b');
       elseif (type(i) == 'R' || type(i) == 'L' || type(i) == 'S')
           ddx=[];
           ddy=[];
           ddyaw=[];
            for i=1:length(dx)
               dx_1= cos(-start_yaw) * dx(i) + sin(-start_yaw)* dy(i) + start_x;
               ddx=[ddx dx_1];
               dy_1= -sin(-start_yaw) * dx(i) + cos(-start_yaw)* dy(i) + start_y;
               ddy=[ddy dy_1];
               dyaw_1=pi_2_pi(t(i)+ start_yaw);
               ddyaw = [ddyaw dyaw_1];
            end
            x = [x,ddx];
            y = [y,ddy];
            angle=[angle,ddyaw];
          plot(ddx,ddy,'r');
       else
           %nothing
       end
        hold on
    end
    axis equal
    plot(start_x,start_y,'kx','LineWidth',2,'MarkerSize',10)
    plot(end_x,end_y,'ko', 'LineWidth',2,'MarkerSize',10)
%     veh = plot(x(1),y(1),'d','MarkerFaceColor','g','MarkerSize',10);
    videoFWriter = VideoWriter('Parking1.mp4','MPEG-4');
    open(videoFWriter);
    [vehx,vehy] = getVehTran(x(1),y(1),angle(1)); % 根据后轴中心的位姿计算车辆边框的位姿
    h1 = plot(vehx,vehy,'r','LineWidth',4); % 车辆边框
    h2 = plot(x(1),y(1),'rx','MarkerSize',10); % 车辆后轴中心
    hold off
    pause(1)
    for k = 2:length(x)
        veh.XData = x(k);
        veh.YData = y(k);
        angle_2=angle(k);
        dl = norm([x(k)-x(k-1),y(k)-y(k-1)]);
        [vehx,vehy] = getVehTran(veh.XData,veh.YData,angle_2);
        h1.XData = vehx; % 更新h1图像句柄,把车辆边框四个角点的x坐标添加进去
        h1.YData = vehy;
        h2.XData = veh.XData; % 更新h2图像句柄,把车辆边框四个角点的y坐标添加进去
        h2.YData = veh.YData;
        img = getframe(gcf);
        writeVideo(videoFWriter,img)
        if(x(end)==end_x)
             close(writerObj); %// 关闭视频文件句柄  
        end
            
    end    
end


 % 根据后轴中心的位姿计算车辆边框的位姿
function [x,y] = getVehTran(x,y,theta)
    W = 0.4;
    LF = 0.1;
    LB = 0.1;

    % 车辆的边框由四个角点确定
    Cornerfl = [LF, W/2]; % 左前方角点
    Cornerfr = [LF, -W/2]; % 右前方角点
    Cornerrl = [-LB, W/2]; % 左后方角点
    Cornerrr = [-LB, -W/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
     
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
%     patch(x,y,[1 1 0])
end
function ang=pi_2_pi(angle)
    ang= (angle + pi); % (2 * math.pi) - math.pi
end


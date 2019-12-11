clc
close all
clear all

%load Waypoints and State Constraint points
load('WayP.mat')
X=dd(:,1);
Y=dd(:,2);
Z=dd(:,3);
% load('Constraints.mat')
% Define timestep
Ts = 0.1;

waypoints = [X,Y,Z]';

%define initial heading angle
% heading = atan((Y(2)-Y(1)) / (X(2) - X(1)));
%define end target
final = waypoints(:,end);

%set initial localization
z0 = [X(1); Y(1); Z(1); zeros(9,1)]; % initial heigth of 0 for now
% z0 = zeros(12,1);
z  = z0;
z_list = z;

% %set reference velocity
% v_ref = 5;

% Define horizon
N = 50;
i = 0;

%while the model has not reached within a certain tolerance of the end
%point
%while norm(z(1:3) - final) > 2
for M=1:1000
    current_dis = vecnorm(waypoints-z(1:3), 2,1);
%     current_idx = find(current_dis == min(current_dis));
    [val,current_idx] = min(current_dis);
    goal_idx = current_idx + 5;
%     dist=z0(3)*Ts;
%     x_pos=z0(1)+cos(z0(4))*dist;
%     y_pos=z0(2)+sin(z0(4))*dist;
%     for i =current_idx+1:length(waypoints)
%         x=waypoints(1,i);
%         y=waypoints(2,i);
%         if x>x_pos && y>y_pos
%             goal_idx=current_idx+i;
%             break
%         end
%     end
    % Define input constraints (force of each propeller)
    umax = [9000 9000 9000 9000]';
    umin = [0 0 0 0]';
    
    % Define goal state constraints (X,Y,V,Heading)
    goal = [waypoints(:, goal_idx)];
    disp(['Goal Index:', num2str(goal_idx)])
% %     
% %     %calculate distance between points
% %     d_x=v_ref*Ts;
% %     
% %     %calculate next n-points to interpolate
% %     x_interp=[];
% %     y_interp=[];
% %     angle=atan((goal(2)-z0(2))/(goal(1)-z0(1)));
% %     for j=1:N
% %         x_interp=[x_interp z0(1)+cos(angle)*j*d_x];
% %         y_interp=[y_interp z0(2)+sin(angle)*j*d_x];
% %     end
% %     for k=1:N
% %         zN(:,k) = [x_interp(k); y_interp(k); v_ref];
% %     end
    
    % Define constraints on roll, pitch, and roll, pitch derivatives (not currently defining
    % constraint for velocity)
    zMax = [15 15 10 10]';
    zMin = [-15 -15 -10 -10]';
    
    disp(['Currently Solving for iter:', num2str(i)])
    [feas, zOpt, uOpt, JOpt] = CFTOC(N, z0, goal, zMin, zMax, umin, umax, Ts);
    
    u = uOpt(:, 1);
    z = zOpt(:, 2);
    z_list = [z_list z];
    z0 = z;
    
    i = i + 1;
end


function [feas, zOpt, uOpt, JOpt] = CFTOC(N, z0, zN, zmin, zmax, umin, umax, Ts)

% Define state matrix
z = sdpvar(12,N+1);

% Define input decision variables
u = sdpvar(4,N);

% %define lane constraints
% buff=0;
% lane_width=10;
% quadcopter_width=2;

P = eye(3);
Q = eye(3);
R = 10*eye(4);

%define objective function
objective=0;

objective = (z(1:3,N+1) - zN(1:3))'*P*(z(1:3,N+1) - zN(1:3));
for j=1:N
    objective = objective + (z(1:3,j) - zN(1:3))'*Q*(z(1:3,j) - zN(1:3)); % removed velocity goal
    objective = objective + u(:,j)'*R*u(:,j);
end

%define state and input constraints
constraints = [];
constraints = [constraints z(:,1)==z0];
for i = 1:N
    % 	    constraints = [constraints zmin<=z(:,i)<=zmax];
%     %find the absolute value of the current position from the reference
%     dist_ref=abs((z(1,i)-zN(1,i))^2+(z(2,i)-zN(2,i))^2+(z(3,i)-zN(3,i))^2);
%     constraints=[constraints (dist_ref+buff+quadcopter_width/2)<=lane_width/2];
    constraints = [constraints umin<=u(:,i)<=umax];
    constraints = [constraints z(:,i+1) == linearDynamicsQuadcopterDiscrete(z(:,i), u(1,i), u(2,i), u(3,i), u(4,i), Ts)];
%     if i <= N-1
%         % 	        constraints = [constraints -pi/10<=u(2,i+1)-u(2,i)<=pi/10];
%     end
end
for k=1:N+1
    constraints=[constraints zmin(1:2)<=z(7:8,k)<=zmax(1:2) zmin(3:4)<=z(10:11,k)<=zmax(3:4)];
end
constraints=[constraints z(1:3,N+1) == zN];

% Set options for YALMIP and solver
options = sdpsettings('verbose', 0, 'solver', 'quadprog');
% Solve
sol = optimize(constraints, objective, options);
if sol.problem == 0
    feas = 1;
    zOpt = value(z);
    uOpt = value(u);
    JOpt = value(objective);
else
    feas=0;
    zOpt = [];
    uOpt = [];
    JOpt = value(objective);
end
end
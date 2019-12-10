clc
close all
clear all

%load Waypoints and State Constraint points
load('Waypoints_new.mat')
% load('Constraints.mat')

% Define timestep
Ts = 0.1;

waypoints = [X; Y];

%define initial heading angle
% heading = atan((Y(2)-Y(1)) / (X(2) - X(1)));
%define end target
final = waypoints(:,end);

%set initial localization
z0 = [X(1); Y(1); 0; zeros(9,1)]; % initial heigth of 0 for now
% z0 = zeros(12,1);
z  = z0;
z_list = z;

%set reference velocity
v_ref = 5;

% Define horizon
N = 10;
i = 0;

%while the model has not reached within a certain tolerance of the end
%point
while norm(z(1:2) - final) > 2
	current_dis = vecnorm(waypoints-z(1:2), 2, 1);
	current_idx = find(current_dis == min(current_dis));
	goal_idx = current_idx + 2;
    
    % Define input constraints
    umax = [100 100 100 100]';
    umin = [0 0 0 0]';
    
    % Define goal state constraints (X,Y,V,Heading)
    goal = [waypoints(:, goal_idx); v_ref];
    disp(['Goal Index:', num2str(goal_idx)])

    % Define constraints
    zMax = 10*ones(12,1);
    zMin = -10*ones(12,1);

    disp(['Currently Solving for iter:', num2str(i)])
    [feas, zOpt, uOpt, JOpt] = CFTOC(N, z0, goal, zMin, zMax, umin, umax, Ts);
    
    u = uOpt(:, 1);
    z = zOpt(:, 2);
    z_list = [z_list z];
    z0 = z;

    i = i + 1;
end


function [feas, zOpt, uOpt, JOpt] = CFTOC(N, z0, goal, zmin, zmax, umin, umax, Ts)

	% Define state matrix
	z = sdpvar(12,N+1);

	% Define input decision variables
	u = sdpvar(4,N);

	%define objective function
	objective=0;

	for j=1:N
		objective = objective + (z(1, j) - goal(1))^2 + (z(2, j) - goal(2))^2; % removed velocity goal
		objective = objective + u(1, j)^2 + u(2,j)^2;
    end  

	%define state and input constraints
	constraints = [];

	for i = 1:N
% 	    constraints = [constraints zmin<=z(:,i)<=zmax];
	    constraints = [constraints umin<=u(:,i)<=umax];
	    constraints = [constraints z(:,i+1) == linearDynamicsQuadcopterDiscrete(z(:,i), u(1,i), u(2,i), u(3,i), u(4,i), Ts)];
	    if i <= N-1
% 	        constraints = [constraints -pi/10<=u(2,i+1)-u(2,i)<=pi/10];
	    end
	end

	constraints = [constraints z(:,1)==z0];%, zmin<=z(:,N+1)<=zmax];

	% Set options for YALMIP and solver
	options = sdpsettings('verbose', 0, 'solver', 'ipopt');
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
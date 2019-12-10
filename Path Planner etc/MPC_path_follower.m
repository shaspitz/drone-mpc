  clear('all');

%load Waypoints and State Constraint points
load('Waypoints_new.mat')
load('Constraints.mat')

% Define timestep
TS = 0.2;

waypoints = [X; Y];

%define initial heading angle
heading = atan((Y(2)-Y(1)) / (X(2) - X(1)));
%define end target
final = waypoints(:,end);

%set initial localisation
z0     = [X(1); Y(1); 0; heading];
z      = z0;
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
	goal_idx    = current_idx + 2;
    
    % Define input constraints
    umax = [ 3  pi/4]';
    umin = [-3 -pi/4]';
    
    % Define goal state constraints (X,Y,V,Heading)
    goal = [waypoints(:, goal_idx); v_ref];
    disp(['Goal Index:', num2str(goal_idx)])

    % Define constraints
    zMax = [ 1000  1000  8  2*pi]';
    zMin = [-1500 -1500 -2  -2*pi]';

    disp(['Currently Solving for iter:', num2str(i)])
    [feas, zOpt, uOpt, JOpt] = CFTOC(N, z0, goal, zMin, zMax, umin, umax, TS);
    
    u = uOpt(:, 1);
    z = zOpt(:, 2);
    z_list = [z_list z];
    z0 = z;

    i = i + 1;
end


function [feas, zOpt, uOpt, JOpt] = CFTOC(N, z0, goal, zmin, zmax, umin, umax, TS)

	% Define state matrix
	z = sdpvar(4,N+1);
	% assign(z(:,1),z0);

	% Define input decision variables
	u = sdpvar(2,N);

	%define objective function
	objective=0;

	for j=1:N
		objective = objective + 5*(z(1, j) - goal(1))^2 + 5*(z(2, j) - goal(2))^2 + 1 * (z(3, j) - goal(3))^2;
		objective = objective + 0.1 * u(1, j)^2 + 0.1 * u(2,j)^2;
    end  

	%define state and input constraints
	constraints = [];

	for i = 1:N
	    constraints = [constraints zmin<=z(:,i)<=zmax];
	    constraints = [constraints umin<=u(:,i)<=umax];
	    constraints = [constraints z(:,i+1) == bikeFE(z(:,i),u(:,i))];
	    if i <= N-1
	        constraints = [constraints -pi/10<=u(2,i+1)-u(2,i)<=pi/10];
	    end
	end

	constraints = [constraints z(:,1)==z0, zmin<=z(:,N+1)<=zmax];

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
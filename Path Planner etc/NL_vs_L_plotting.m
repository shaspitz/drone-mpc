clc
close all
clear all

%load Waypoints and State Constraint points
X=12;
Y=12;
Z=12;

% Define timestep
Ts = 0.1;

waypoints = [X,Y,Z]';
zN(1:3,1) = waypoints;
zN(4:6,1) = [0;0;0];
zN(7:9,1) = [0;0;0];
zN(10:12,1) = [0;0;0];

%set initial localization
z = zeros(12,1);

% Define horizon
N = 30;
i = 0;

% Define cell arrays to store open loop trajectories
L_openloop_z = [];
L_openloop_u = [];
L_openloop_J = [];
NL_openloop_z = [];
NL_openloop_u = [];
NL_openloop_J = [];

%%   
umax = [9000 9000 9000 9000]';
umin = [0 0 0 0]';
zMin = [];
zMax = [];
  
[feas, zOpt, uOpt, JOpt] = CFTOC_L(N, z, zN, zMin, zMax, umin, umax, Ts);
if feas == 0
    disp("ERROR IN LINEAR YALMIP CFTOC");
end

L_openloop_z = zOpt;
L_openloop_u = uOpt;
L_openloop_J = JOpt;

% [feas, zOpt, uOpt, JOpt] = CFTOC_NL(N, z, zN, zMin, zMax, umin, umax, Ts);
% if feas == 0
%     disp("ERROR IN NONLINEAR YALMIP CFTOC");
% end
% 
% NL_openloop_z = zOpt;
% NL_openloop_u = uOpt;
% NL_openloop_J = JOpt;

z0 = zeros(12,1);
for i = 1:N
    z0(:,i+1) = nonlinearDynamicsQuadcopterDiscrete(z0(:,i), L_openloop_u(1,i), L_openloop_u(2,i), L_openloop_u(3,i), L_openloop_u(4,i), Ts);
end

NL_openloop_z = z0;

%% Plotting

% X direction
close all;
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(1,:));
hold on;
plot(t_vec,NL_openloop_z(1,:));
legend({'Linear' 'Nonlinear'})
title('X Position vs Time');

% Velocity X 
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(4,:));
hold on;
plot(t_vec,NL_openloop_z(4,:));
legend({'Linear' 'Nonlinear'})
title('Velocity X vs Time');

% Velocity Y
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(5,:));
hold on;
plot(t_vec,NL_openloop_z(5,:));
legend({'Linear' 'Nonlinear'})
title('Velocity Y vs Time');

% Velocity Z
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(6,:));
hold on;
plot(t_vec,NL_openloop_z(6,:));
legend({'Linear' 'Nonlinear'})
title('Velocity Z vs Time');

% Roll 
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(7,:));
hold on;
plot(t_vec,NL_openloop_z(7,:));
legend({'Linear' 'Nonlinear'})
title('Roll');

% Pitch 
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(8,:));
hold on;
plot(t_vec,NL_openloop_z(8,:));
legend({'Linear' 'Nonlinear'})
title('Pitch');

% Yaw 
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(9,:));
hold on;
plot(t_vec,NL_openloop_z(9,:));
legend({'Linear' 'Nonlinear'})
title('Yaw');

% Roll Rate
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(10,:));
hold on;
plot(t_vec,NL_openloop_z(10,:));
legend({'Linear' 'Nonlinear'})
title('Roll Rate');

% Pitch Rate
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(11,:));
hold on;
plot(t_vec,NL_openloop_z(11,:));
legend({'Linear' 'Nonlinear'})
title('Pitch Rate');

% Yaw Rate
figure();
t_vec = 0:Ts:N*Ts;
plot(t_vec,L_openloop_z(12,:));
hold on;
plot(t_vec,NL_openloop_z(12,:));
legend({'Linear' 'Nonlinear'})
title('Yaw Rate');

%%

function [feas, zOpt, uOpt, JOpt] = CFTOC_L(N, z0, zN, zmin, zmax, umin, umax, Ts)

% Define state matrix
z = sdpvar(12,N+1);

% Define input decision variables
u = sdpvar(4,N);

P = eye(12);
Q = 10*eye(12);
R = eye(4);

%define objective function
objective=0;

objective = (z(:,N+1) - zN(:))'*P*(z(:,N+1) - zN(:));
for j=1:N
    objective = objective + (z(:,j) - zN(:))'*Q*(z(:,j) - zN(:));
    objective = objective + u(:,j)'*R*u(:,j);
end

%define state and input constraints
constraints = [];
constraints = [constraints z(:,1)==z0];
for i = 1:N
    constraints = [constraints umin<=u(:,i)<=umax];
    constraints = [constraints z(:,i+1) == linearDynamicsQuadcopterDiscrete(z(:,i), u(1,i), u(2,i), u(3,i), u(4,i), Ts)];
end

constraints=[constraints z(:,N+1) == zN(:)];

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

function [feas, zOpt, uOpt, JOpt] = CFTOC_NL(N, z0, zN, zmin, zmax, umin, umax, Ts)

% Define state matrix
z = sdpvar(12,N+1);

% Define input decision variables
u = sdpvar(4,N);

P = eye(12);
Q = 10*eye(12);
R = eye(4);

%define objective function
objective=0;

objective = (z(:,N+1) - zN(:))'*P*(z(:,N+1) - zN(:));
for j=1:N
    objective = objective + (z(:,j) - zN(:))'*Q*(z(:,j) - zN(:));
    objective = objective + u(:,j)'*R*u(:,j);
end

%define state and input constraints
constraints = [];
constraints = [constraints z(:,1)==z0];
for i = 1:N
    constraints = [constraints umin<=u(:,i)<=umax];
    constraints = [constraints z(:,i+1) == nonlinearDynamicsQuadcopterDiscrete(z(:,i), u(1,i), u(2,i), u(3,i), u(4,i), Ts)];
end

constraints=[constraints z(:,N+1) == zN(:)];

% Set options for YALMIP and solver
options = sdpsettings('verbose', 0, 'solver', 'IPOPT');
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


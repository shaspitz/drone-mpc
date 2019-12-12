function [dx] = curvilinearDynamicsQuadcopter(x, cp1, cp2, cp3, cp4)

% Linear Dynamics of a Quadcopter following Mueller's Notes
% Linearizing about hover
% Using small angle assumption and small angular velocity
% Input x is the current state vector
% Inputs cp1, cp2, cp3, cp4 are the scalar forces of each propeller (parallel to
% the body-fixed 3B axis)
% Output dx is the first derivative of the current state vector
% Unraveling the input state vector

s = x(1);         % Position on reference curve       
dy = x(2);        % Vertical distance from S to COM
dz = x(3);        % Horizontal distance from S to COM

% Velocity of the quadcopter’s COM relative to the earth-fixed frame, expressed in
% the earth-fixed coordinate system
v1 = x(4);
v2 = x(5);
v3 = x(6);

% Orientation of the body-fixed frame with respect to the earth-fixed frame
% Using Euler Angles for simplicity - this might need to be changed later

phi   = x(7);     % Roll
theta = x(8);     % Pitch
psi   = x(9);     % Yaw

% Angular velocity of the body-fixed frame with respect to the earth-fixed frame, expressed
% in the body-fixed coordinate system

p = x(10);        % roll rate
q = x(11);        % pitch rate
r = x(12);        % yaw rate

ed = x(13);       % Direction angle error
ei = x(14);       % Inclination angle error

% Get necessary parameters
[Jxx, Jzz, m, l, k] = quadcopterParameters();
[cSigma, n1, n2, n3] = mixerMatrix(cp1, cp2, cp3, cp4);
g = 9.81;
deltacSigma = cSigma - m*g;
V = sqrt(v1^2 + v2^2 + v3^2);
% c   = ???
% tau = ???

% Derivative of position is velocity
ds  = V*cos(ed)*cos(ei)/(1-c*dy);
ddy = V*sin(ed)*cos(ei) + tau*dz*ds;
ddz = -V*sin(ei) - tau*dy*ds;

% Derivative of velocity
dv1 = theta * g;
dv2 = -phi * g;
dv3 = deltacSigma/m;

% Derivative of orientation using Euler Angles
dphi   = p;     
dtheta = q;     
dpsi   = r;

% Derivative of angular velocity
dp = n1/Jxx;
dq = n2/Jxx;
dr = n3/Jzz;

% Derivative of Direction angle error and Inclination angle error
ded = r/cos(ei) - tau*ds*tan(ei)*cos(ed) - c*ds;
dei = tau*ds*sin(ed) + q;

% Pack into state derivative vector
dx = [ds;
      ddy;
      ddz;
      dv1;
      dv2;
      dv3;
      dphi;
      dtheta;
      dpsi;
      dp;
      dq;
      dr;
      ded;
      dei];
  
end

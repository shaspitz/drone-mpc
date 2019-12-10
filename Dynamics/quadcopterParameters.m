function [Jxx, Jzz, m, l, k] = quadcopterParameters()

% Quadcopter physical parameters
r = 0.2;        % radial distance of propeller from COM
l = r/sqrt(2);
mB = 1;         % Mass of the electronics/battery/ect located at the COM
mM = 0.1;       % Mass of the motors
m = mB + 4*mM;  % Total mass of the quadcopter
k = 5;          % Proportionality constant

% Locations of the motors in the body fixed frame
s1 = [l; l; 0];
s2 = [l; -l; 0];
s3 = [-l; -l; 0];
s4 = [-l; l;0];

s = [s1, s2, s3, s4];

% Calculating the inertial tensor
[J] = computeJ(s,mM);
Jxx = J(1,1);
Jzz = J(3,3);

end


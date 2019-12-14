%% Trajectory
% This scripts generates the interpolated data for the reference trajectory
close all; clc; clear all;

% 32187 is 20mi
r.x = 32187;
r.y = 32187;
r.z = 32187;

% theta = linspace(0,2*pi,360);
% x = r.x*cos(theta);
% y = r.y*sin(theta);
% z = r.z*sin(theta);
% circlewaypoints = [x',y',z'];
% plot3(x,y,z)
% grid on
% axis square

WP =...
    [0 0 0;
    25 0 40;
    40 0 50;
    60 0 50;
    80 0 40;
    100 0 35;
    110 10 35;
    120 20 35;
    120 35 35;
    125 40 35;
    125 50 35; 
    120 40 35;
    120 80 30;
    115 60 50;
    110 40 50;
    110 25 25];
 
WP = 10*WP; 
temp = cat(1,0,cumsum(sqrt(sum(diff(WP,[],1).^2,2))));
interpWP = interp1(temp, WP, unique([temp(:)' linspace(0,temp(end),200)]),'cubic');

dd = interpWP;

save 'waypoints_lesscurve.mat' dd

% 
% figure, hold on
% plot3(WP(:,1),WP(:,2),WP(:,3),'.b-')
% plot3(interpWP(:,1),interpWP(:,2),interpWP(:,3),'.r-')
% 
% grid on
% title('Quadcopter Trajectoy')
% axis image, view(3), legend({'Original','Interp. Spline'})
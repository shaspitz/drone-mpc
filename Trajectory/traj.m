%% Trajectory
% This scripts generates the interpolated data for the reference trajectory
close all; clc; clear all;

% 32187 is 20mi
% r.x = 32187;
% r.y = 32187;
% r.z = 32187;
% 
% theta = linspace(0,2*pi,360);
% x = r.x*cos(theta);
% y = r.y*sin(theta);
% z = r.z*sin(theta);
% circlewaypoints = [x',y',z'];
% plot3(x,y,z)
% grid on
% axis square

load('exportWP')

for i = 0:length(exportWP)-1
    WP(i+1,:) = exportWP(length(exportWP)-i).Position
end

for i = 1:length(exportWP)-1
    distWP(i) = sqrt((WP(i+1,1)-WP(i,1))^2+(WP(i+1,2)-WP(i,2))^2);
end

WP(12,:) = WP(1,:);
WP(:,3) = zeros(12,1);
WP(:,3) = ...
    [0;
    100;
    100;
    0;
    100;
    100;
    0;
    100;
    100;
    100;
    100;
    0];
WP = 5000/700*WP;

temp = cat(1,0,cumsum(sqrt(sum(diff(WP,[],1).^2,2))));
interpWP = interp1(temp, WP, unique([temp(:)' linspace(0,temp(end),1430)]),'PCHIP');
dd = interpWP;

% delete last 3 waypoints to fix starting failure
dd = dd(1:end-80,:);

save 'WP_map.mat' dd
% 
figure, hold on
% open('BayAreaMap.fig')
plot3(WP(:,1),WP(:,2),WP(:,3),'.b-')
plot3(dd(:,1),dd(:,2),dd(:,3),'.r-')


grid on
title('Quadcopter Trajectoy')
axis image, view(3), legend({'Original','Interp. Spline'})
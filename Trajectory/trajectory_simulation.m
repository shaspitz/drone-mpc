function Y=trajectory_simulation()
%load('U.mat')
load('Z.mat')
%load('OpenLoopPred.mat')
openfig('BayAreaMap_refTraj.fig') 
legend off
    hold on
for i=1:length(z_list)    
    g=plot3(z_list(1,:)/7, z_list(2,:)/7, z_list(3,:)/7,'bo-', 'linewidth', 2)
    pause(0.1)
end
    X=1;
end
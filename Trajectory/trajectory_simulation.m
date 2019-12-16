function Y=trajectory_simulation()
load('Z.mat')
fig=openfig('BayAreaMap3D.fig');
hold on
legend off
for i=1:length(z_list)
    g=plot3(z_list(1,i)/(5000/700), z_list(2,i)/(5000/700), z_list(3,i)/(5000/700),'k.-', 'MarkerSize', 5)
    h=plot3(z_list(1,i)/(5000/700), z_list(2,i)/(5000/700), z_list(3,i)/(5000/700),'g*', 'MarkerSize', 10)
    pause(0.1)
    delete (h)
end
X=1;


openfig('BayAreaMap2D.fig');
hold on
legend off
for i=1:length(z_list)
    g=plot3(z_list(1,i)/(5000/700), z_list(2,i)/(5000/700), z_list(3,i)/(5000/700),'k.-', 'MarkerSize', 5)
    h=plot3(z_list(1,i)/(5000/700), z_list(2,i)/(5000/700), z_list(3,i)/(5000/700),'g*', 'MarkerSize', 10)
    pause(0.1)
    delete (h)
end
X=1;
end
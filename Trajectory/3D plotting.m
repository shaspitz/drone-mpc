function Y=trajectory_simulation()

load('z_list_clean.mat')

writeObj = VideoWriter('3DMPCdrone.avi');
open(writeObj);

fig=openfig('BayAreaMap3D.fig');
hold on
legend off
xlabel('x')
ylabel('y')
zlabel('z')
title('3D MPC Simulation - UAV')
for i=1:length(z_list_clean)
    g=plot3(z_list_clean(1,i)/(5000/700), z_list_clean(2,i)/(5000/700), z_list_clean(3,i)/(5000/700),'k.-', 'MarkerSize', 5)
    h=plot3(z_list_clean(1,i)/(5000/700), z_list_clean(2,i)/(5000/700), z_list_clean(3,i)/(5000/700),'b*','LineWidth',5, 'MarkerSize', 20)
    view(180,20)
    frame = getframe(gcf);
    writeVideo(writeObj,frame);
    pause(0.1)
    delete (h)
end
X=1;


% openfig('BayAreaMap2D.fig');
% hold on
% legend off
% title('3D MPC Simulation - UAV')
% 
% for i=1:length(z_list_clean)
%     g=plot3(z_list_clean(1,i)/(5000/700), z_list_clean(2,i)/(5000/700), z_list_clean(3,i)/(5000/700),'k.-', 'MarkerSize', 5)
%     h=plot3(z_list_clean(1,i)/(5000/700), z_list_clean(2,i)/(5000/700), z_list_clean(3,i)/(5000/700),'gsquare', 'LineWidth',5,'MarkerSize', 20)
%     frame = getframe(gcf);
%     writeVideo(writeObj,frame);
%     pause(0.1)
%     delete (h)
%     
%     
% end
% X=1;
close(writeObj);
end

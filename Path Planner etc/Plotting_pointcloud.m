ptCloud = pcread('map.pcd');
%pcshow(ptCloud);
roi=[-1.8197243e+02,99.7775803,-1.2216548e+02,68.1845169,4,10];
indices = findPointsInROI(ptCloud,roi);
[ptCloudOut,ind] = removeInvalidPoints(ptCloud);
ptCloudB = select(ptCloudOut,indices);
%figure
% pcshow(ptCloudOut.Location,[0.5 0.5 0.5]);
% hold on
% pcshow(ptCloudB.Location,'r');
% legend('Point Cloud','Points within ROI','Location','southoutside','Color',[1 1 1])
% hold off
% figure
% pcshow(ptCloudB)  
locations = ptCloudB.Location;
%calculate waypoints
x_pts=linspace(-126.05,-61.77,20);
y_pts=linspace(-18.42,-8.519,20);
x_pts1=linspace(-21.72,-61.77,30);
y_pts1=linspace(-96.22,-8.519,30);
X=[x_pts x_pts1];
Y=[y_pts y_pts1];
%calculate position constraints
con_x_max=linspace(-124.3,-60.77,20);
con_y_max=linspace(-10.665,-4.265,20);
con_x_min=linspace(-128.3,-66.41,20);
con_y_min=linspace(-21.87,-13.82,20);

con_x_max1=linspace(-20.27,-58.22,30);
con_y_max1=linspace(-92.17,-3.57,30);
con_x_min1=linspace(-28.12,-66.41,30);
con_y_min1=linspace(-98.77,-13.82,30);
X_con_max=[con_x_max con_x_max1];
Y_con_max=[con_y_max con_y_max1];
X_con_min=[con_x_min con_x_min1];
Y_con_min=[con_y_min con_y_min1];
hold on
grid on
%plot obstacles
plot(locations(:,1), locations(:,2),  '.')
%plot potential routes
% plot([-126.05,19.355],[-18.42,4.01],'r-')
% plot([-106.5,31.305],[-77.645,-14.8175],'r-')
% plot([-21.72,-63.67],[-96.22,-4.365],'r-')
%plot waypoints
plot(X,Y,'r.')
%plot X,Y constraints
plot(X_con_max,Y_con_max,'g.')
plot(X_con_min,Y_con_min,'g.')
%save X,Y coordinates of waypoints and X,Y constraints
save Constraints X_con_max Y_con_max X_con_min Y_con_min
save Waypoints X Y
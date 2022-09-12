function [Percentage,Vol_point_in_ROI] = Percentage_within_ROI(workspace_trans_remove_OutL,b_value)

%Calculate the Volume of ROI (Cylinder)
r = 150; %in mm 
h = 200; %in mm 
Vol_Cyl = pi*r^2*h; %in mm3

%%Calculate pointcloud within Cylinder (ROI)
x_coord =  workspace_trans_remove_OutL(:,1);
y_coord =  workspace_trans_remove_OutL(:,2);
z_coord =  workspace_trans_remove_OutL(:,3);
POI = -(b_value/2); 
height_upper = POI + h/2; %endeffector point +100mm above 
height_lower = POI - h/2; %endeffector point +100mm above 

cylinder_logical_xy = (x_coord).^2 + (y_coord).^2 <= r^2; %x2 +y2 = r2 cylinder formula
cylinder_logical_z = height_lower <= z_coord & z_coord <= height_upper; %z-coordinate within the heigh of cylinder
cylinder_logical_xyz = cylinder_logical_xy & cylinder_logical_z; %points logical that fulfil all conditions

ptCloud_within_ROI = workspace_trans_remove_OutL(cylinder_logical_xyz,:); 
% plot3(ptCloud_within_ROI(:,1),ptCloud_within_ROI(:,2),ptCloud_within_ROI(:,3),'.g')


%Calculate the Volume of pointcloud in ROI 
%% Plot Boundary in 3D
results = ptCloud_within_ROI;
[k,Vol_point_in_ROI] = boundary(results, 0.2); %in mm3 shrink factor = 0 =convexhull,cause no more outliers exist. 
trisurf(k,results(:,1),results(:,2),results(:,3),'FaceColor','blue','FaceAlpha',0.1)

%Percentage 
Percentage =  (Vol_point_in_ROI/Vol_Cyl) *100;







end
% Alternative 
% ptCloud = pointCloud(workspace_trans_remove_OutL); %create pointcloud 
% ROI = [-r r -r r -h-200 -h]; %define the x_min x_max, y_min y_max and z_min z_max limit of ROI (CUBE SHAPE! NOT CYLINDER) 
% indices = findPointsInROI(ptCloud,ROI); 
% ptCloud_within_ROI = select(ptCloud,indices);

%% for Visualisation purpose  
% figure
% pcshow(ptCloud.Location,[0 1 0]) %colour green
% set(gcf,'color','w');
% set(gca,'color','w');
% hold on
% pcshow(ptCloud_within_ROI.Location,'r');
% legend('Point Cloud','Points within ROI','Location','southoutside','Color',[1 1 1])
% hold off


function [convexhull_volume, workspace_trans_mat_total] = convexhull_khaw(workspace_trans_mat_total)
%UNTITLED2 Summary of this function goes here

%% Change the values of NaN to 0 (so that convhull function is applicable) 
indices = isnan(workspace_trans_mat_total) == 1;
workspace_trans_mat_total(indices) = 0; 

[k, convexhull_volume] = convhull(workspace_trans_mat_total,'Simplify',true);

% Plot the convexhull Volume 
figure
trisurf(k,workspace_trans_mat_total(:,1),workspace_trans_mat_total(:,2),workspace_trans_mat_total(:,3),'FaceColor','b','Edgecolor','b')
hold on 

%% Define the 3D projection onto the wall 
y_plane = max(workspace_trans_mat_total(:,2))+150; %from maximum of y_plane coordinate +200 mm (show on right)
x_plane = max(workspace_trans_mat_total(:,1))+150; %from maximum of x_plane coordinate +200 mm (show on left)
z_plane = max(workspace_trans_mat_total(:,2))-800; %from maximum of z_plane coordinate -150 mm (show on bottom) 

trisurf(k,workspace_trans_mat_total(:,1), y_plane*ones(size(workspace_trans_mat_total(:,2))), workspace_trans_mat_total(:,3),'FaceColor','r','Edgecolor','r'); % project in x-z axis at y=300
trisurf(k,x_plane*ones(size(workspace_trans_mat_total(:,1))), workspace_trans_mat_total(:,2), workspace_trans_mat_total(:,3),'FaceColor','c','Edgecolor','c'); % project in y-z axis at x=2
trisurf(k,workspace_trans_mat_total(:,1), workspace_trans_mat_total(:,2), z_plane*ones(size(workspace_trans_mat_total(:,3))),'FaceColor','g','Edgecolor','g'); % project in x-y axis at z=-2

% add Title workaround methode
formatSpec = "The current workspace is: %e %s";
format long g
current_ws = convexhull_volume*1e-9; %1e-9 for changing from mm3 to m3
unit = 'm3';
str = sprintf(formatSpec,current_ws,unit);
title(str)
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate

end
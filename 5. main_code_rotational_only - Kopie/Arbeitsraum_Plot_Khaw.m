function [workspace_pointwise_trans] = Arbeitsraum_Plot_Khaw(workspace_logical,coordinate,POI_rot,a,b)
%Plot the feasible workspace here

%   Detailed explanation goes here
sz = [length(coordinate.x), length(coordinate.y), length(coordinate.z)]; %define the size of matrix for command ind2sub  
index_convexhull_point = find(workspace_logical == 1);

if isempty(index_convexhull_point)
%     disp('No ws_point exist')
    workspace_pointwise = [NaN NaN NaN]; %select x-coordinate from workspace 
else
     for j = 1 : length(index_convexhull_point)
            [id_x, id_y, id_z] = ind2sub(sz, index_convexhull_point(j)); %Convert linear indices to subscripts with dimension of grid_size(67)
            workspace_pointwise(j, 1) = coordinate.x(id_x); %select x-coordinate from workspace 
            workspace_pointwise(j, 2) = coordinate.y(id_y); %select y-coordinate from workspace 
            workspace_pointwise(j, 3) = coordinate.z(id_z); %select z-coordinate from workspace 
     end
     
end

workspace_pointwise_trans = workspace_pointwise + POI_rot'; %translation of the results to POI

% plot3(workspace_pointwise(:,1), workspace_pointwise(:,2),workspace_pointwise(:,3), '.r','LineWidth',8); %original at trocar point
% hold on 
plot3(workspace_pointwise_trans(:,1), workspace_pointwise_trans(:,2),workspace_pointwise_trans(:,3), '.g','LineWidth',8); %ws at POI (end of the rod)
hold on 
daspect([1,1,1]) %For equal data unit lengths in all directions

%% Plot Trocar point at Origin
plot3(0,0,0,'bo','LineWidth',5)
hold on 

%% Plot Region of Interest (ROI)
r = 150; %radius in mm 
[X,Y,Z] = cylinder(r);
X = X-0;
Y = Y-0;
h = 200; %height in mm
Z = (Z*h)-400; %minus 100 so that its from -100 to 100 in Z-axis
surf(X,Y,Z,'FaceColor','w','FaceAlpha','0.3')

%% Title of plot
length_frame = max(a(1,:))-min(a(1,:)); %x-axis
width_frame = max(a(2,:))-min(a(2,:)); %y-axis
height_frame = max(a(3,:))-min(a(3,:)); %z-axis 
height_rod = max(b(3,:))-min(b(3,:));

title('Wrench-feasible workspace in Cable-Driven Haptic Device')
txt = ['L= ' int2str(length_frame) ' x W= ' int2str(width_frame) ' x H= ' int2str(height_frame) ' Rod= ' int2str(height_rod) ' [mm]'];
subtitle(txt)
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate

end

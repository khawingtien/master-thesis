function [position_bowl_mat, R_x_cell, R_z_cell, position_bowl_360] = ws_position_bowl()

position = [0;0;-300];
ws_orientation = [];
position_bowl_mat = [];
R_x_mat = [];
R_z_cell = cell(1,360);
% 
% figure
% plot3(position(1),position(2),position(3),'bo') % end of end-effector 
% hold on 
% plot3(0,0,0,'ro') %origin 
% hold on

%Rotation of all orientation around x-axis
    for i=1:30 %orientation 1 to 30°
        rotation = [1 0 0 deg2rad(i)]; %rotation at x-axis
        R_x = axang2rotm(rotation);
        rod_middle = R_x* [[0;0;300], [0; 0; -300]]; %rotation of the center line through origin
%          plot3(rod_middle(1,:),rod_middle(2,:),rod_middle(3,:) ,'b','LineWidth',2) %plot middle line through trocar point
        ws_orientation = [ws_orientation, rod_middle(:,2)]; %append of ws orientation, only with the lower point (POI)
        R_x_mat = [R_x_mat, R_x]; %append of rotation matrix 
        R_x_cell{1,i} = R_x;
    end
z_angle_step=1;
z_angles = 0:z_angle_step:360-z_angle_step;
%Rotation of all the orientation (0 to 360°) around z-axis
for angle = 1:length(z_angles) %%ACHTUNG INTERVAL [with 10° in between 1,11,21...360]
    z_angle=z_angles(angle);
rotation_z = [0 0 1 deg2rad(z_angle)]; %rotation at z-axis  
R_z = axang2rotm(rotation_z); 
    for index = 1:30 %rotation for every Kippwinkel 
    position_bowl = R_z* ws_orientation(:,index);
%         plot3(position_bowl(1,:),position_bowl(2,:),position_bowl(3,:) ,'ro','LineWidth',1)  %plot rotation     
    position_bowl_mat = [position_bowl_mat, position_bowl];
    position_bowl_cell{1,index} = position_bowl; 
    end
R_z_cell{1,angle} = R_z;
position_bowl_360{1,angle} = position_bowl_cell;
end


%save the Rotation matrix in x-axis in cell array 
% RowDist = 3; %distribution in row 
% ColDist = [repmat(3,1,30)]; %distribution in column for total 30 times for (3x30 = total 90)
% R_x_cell = mat2cell(R_x_mat,RowDist,ColDist);

% grid on 
% daspect([1,1,1]) %For equal data unit lengths in all directions
% xlabel('x in mm') %text in x-coordinate
% ylabel('y in mm') %text in y-coordinate
% zlabel('z in mm') %text in z-coordinate

end
%% Function Arbeitsraum
function [workspace_logical,  b_rot_xz, POI_rot, middle_rod] = Arbeitsraum_khaw(a, b, f_min, f_max,noC, rot_axis, rot_name, w_p_x, w_p_t, rotation_w_p, workspace_logical, pulley_kin, R_A, rot_angle_A, coordinate, limit, f_direction)
counter = 1; %predefine counter = 1
% workspace_logical_temp = ones(length(coordinate.x), length(coordinate.y), length(coordinate.z));

%Define Point of Interest (POI) 
POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)

%rotate the POI in accordance with the rotation angle 
% R = axang2rotm(rotation_x); %axis angle to rotation matrix [1 0 0 angle] to Matrix Dimension=(3,3)
% R_z = axang2rotm(rotation_z);
R_z = axang2rotm([rot_axis, deg2rad(rot_name)]);

b_rot = (R_z *b); %maybe dont need anymore. 
% b_rot_xz = R_z* (R *b);
b_rot_xz = R_z*b;
% POI_rot = R_z * (R *POI_offset);
POI_rot = R_z *POI_offset; 
% middle_rod = R_z *(R*[-POI_offset, POI_offset]);
 middle_rod = R_z *([-POI_offset, POI_offset]);

%calculate rotation matrix for wrench 
rotation_matrix.wpx = axang2rotm(rotation_w_p.x); %rotation matrix for f_Y around X-AXIS (IMPORTANT)
rotation_matrix.wpy = axang2rotm(rotation_w_p.y); %rotation matrix for f_X around Y-AXIS (IMPORTANT)

%Go through all the coordinate combination of the x_row, y_column, z_page, and save them in variable workspace_position 
       for i = 1 :length(coordinate.x)
         for j = 1 :length(coordinate.y)
           for k = 1:length(coordinate.z)
             workspace_position = [coordinate.x(i) coordinate.y(j) coordinate.z(k)]'; %workspace_position in a column vector  

        %berechne die Seilkraftverteilung an dieser Position        
        [stop] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, b, f_min, f_max,noC, b_rot,b_rot_xz, w_p_x, w_p_t, rotation_matrix,  limit, f_direction, POI_rot); %hier erstmal nur stop von Interesse tbd
        counter = counter + 1; %no semicolon, to show the current progression during debugging
        workspace_logical(i,j,k) = ~stop;
%         write the value (True or false) into the workspace logical matrix  
%             if stop == 0  %no violation of f_min & f_max (fulfil the requirements)(TRUE)
%               workspace_logical_temp (i,j,k) = 1; %write 1 as TRUE
%             elseif stop == 1 %if violation exist 
%               workspace_logical_temp (i,j,k) = 0; %write 0 as FALSE 
%             end
           end
         end
       end
     
end

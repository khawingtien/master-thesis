%% Function Arbeitsraum
function [workspace_logical, R] = Arbeitsraum_khaw(a, b, f_min, f_max,noC, rotation, w_p_x, w_p_t, rotation_w_p, workspace_logical, pulley_kin, rad_pulley, R_A, rot_angle_A, coordinate, limit, f_direction)
counter = 1; %predefine counter = 1
workspace_logical_temp = ones(length(coordinate.x), length(coordinate.y), length(coordinate.z));

%Define Point of Interest (POI) 
POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)
% POI_offset = [0 0 0]'; %first value of endeffector in z-axis (offset to half the rod length)
%rotate the POI in accordance with the rotation angle 
R = axang2rotm(rotation); %axis angle to rotation matrix [0 1 0 angle] to Matrix Dimension=(3,3)
POI_rot = R * POI_offset; %the position of the POI after rotation at (0,0,0)
 
% %calculate rotation matrix for wrench 
rotation_matrix.wpx = axang2rotm(rotation_w_p.x); %rotation matrix for f_Y around X-AXIS (IMPORTANT)
rotation_matrix.wpy = axang2rotm(rotation_w_p.y); %rotation matrix for f_X around Y-AXIS (IMPORTANT)

% norm_f_V_vector = []; %predefine for norm(f_V) 
%Go through all the coordinate combination of the x_row, y_column, z_page, and save them in variable workspace_position 
       for i = 1:length(coordinate.x)
         for j = 1:length(coordinate.y)
           for k = 1:length(coordinate.z)
             workspace_position = [coordinate.x(i) coordinate.y(j) coordinate.z(k)]'; %workspace_position in a column vector 
             workspace_position = workspace_position - POI_rot;
        %berechne die Seilkraftverteilung an dieser Position 
       
        [stop, R ] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, b, f_min, f_max,noC, R, w_p_x, w_p_t, rotation_matrix, pulley_kin, rad_pulley, R_A, rot_angle_A, limit, f_direction, POI_rot); %hier erstmal nur stop von Interesse tbd
        counter = counter + 1; %no semicolon, to show the current progression during debugging
%         if mod(counter,500)==0
%             disp(counter)
%         end
%         norm_f_V_vector = [norm_f_V_vector ; norm_f_V]; %to show the norm(f_V) value 

        %write the value (True or false) into the workspace logical matrix  
            if stop == 0  %no violation of f_min & f_max (fulfil the requirements)(TRUE)
              workspace_logical_temp (i,j,k) = 1; %write 1 as TRUE
            elseif stop == 1 %if violation exist 
              workspace_logical_temp (i,j,k) = 0; %write 0 as FALSE 
            end
           end
         end
      end
     
%Alt: Fülle 1. Dimension mit gemeinsamer Menge aus 1. und 2. Dimension
%Neu: Compare ws_logical_Temporary und final workspace
workspace_logical = workspace_logical & workspace_logical_temp; %Find logical (AND) between ws_logical_temp and the page before


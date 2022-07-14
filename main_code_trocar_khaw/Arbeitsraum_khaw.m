%% Function Arbeitsraum
function [workspace_logical, R] = Arbeitsraum_khaw(a, b, f_min, f_max, ~, rotation, w_p, w_p_t, rotation_w_p, workspace_logical, pulley_kin, rad_pulley, R_A, rot_angle_A, coordinate)
counter = 1; %predefine counter = 1
workspace_logical_temp = ones(length(coordinate.x), length(coordinate.y), length(coordinate.z));
% offset.x=0;
% offset.y=0;
% offset.z= b(3,5);
% coordinate.z = coordinate.z - min(coordinate.z);


%Go through all the coordinate combination of the x_row, y_column, z_page, and save them in variable workspace_position 
       for i = 1:length(coordinate.x)
         for j = 1:length(coordinate.y)
           for k = 1:length(coordinate.z)
             workspace_position = [coordinate.x(i) coordinate.y(j) coordinate.z(k)]'; %workspace_position in a column vector 
                  
        %berechne die Seilkraftverteilung an dieser Position 
        [stop, R,l] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, b, f_min, f_max, rotation, w_p, w_p_t, rotation_w_p, pulley_kin, rad_pulley, R_A, rot_angle_A); %hier erstmal nur stop von Interesse tbd
        counter = counter + 1 %no semicolon, to show the current progression during debugging 
        
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


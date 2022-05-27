%% Function Arbeitsraum
function [workspace_logical, R] = Arbeitsraum_khaw(a, b, f_min, f_max, grid_n, rotation, w_p, w_p_t, rotation_w_p, workspace, workspace_logical, pulley_kin, rad_pulley, R_A, rot_angle_A, x_row,y_column,z_page)
counter = 1; %predefine counter = 1
stop = 0; %predefine stop = 0
workspace_logical_temp = ones(grid_n+1, grid_n+1, grid_n+1);

%Go through all the coordinate combination of the x_row, y_column, z_page, and save them in variable workspace_position 
       for i = 1: grid_n+1
         for j = 1:grid_n+1
           for k = 1:grid_n+1
             workspace_position = [x_row(i) y_column(j) z_page(k)]'; %workspace_position in a column vector 
                  
        %berechne die Seilkraftverteilung an dieser Position 
        [stop, R,l] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, b, f_min, f_max, rotation, w_p, w_p_t, rotation_w_p, pulley_kin, rad_pulley, R_A, rot_angle_A); %hier erstmal nur stop von Interesse tbd
        counter = counter + 1;
        
        %write the value (True or false) into the workspace logical matrix  
            if stop == 0 %keine Verletzung der f_min, f_max
              workspace_logical_temp (i,j,k) = 1; 
            elseif stop == 1
              workspace_logical_temp (i,j,k) = 0;   
            end
           end
         end
      end
     
%Alt: Fülle 1. Dimension mit gemeinsamer Menge aus 1. und 2. Dimension
%Neu: Compare ws_logical_Temporary und final workspace
workspace_logical = workspace_logical & workspace_logical_temp; %Find logical (AND) between ws_logical_temp and the page before

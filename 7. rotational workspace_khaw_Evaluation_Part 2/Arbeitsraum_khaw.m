%% Function Arbeitsraum
function [workspace_logical,  b_rot_xy, POI_rot, cable_length_mat] = Arbeitsraum_khaw(a, b, f_min, f_max,noC, rotation_axis, rot_angle, w_p, w_p_t, workspace_logical, coordinate, limit)

%Define Point of Interest (POI) 
POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)
lever_arm = POI_offset(3); %only half of rod length 

%Define rotation axis 
R = axang2rotm([rotation_axis, deg2rad(rot_angle)]);  %TODO: write to matrix (save time) 

b_rot_xy = R *b; %Rotation of endeffector 

POI_rot = R *POI_offset; %Rotation of point of interest (the end of endeffector) 

%call the function which calculate the wrench direction coordinate 
wrench_mat = wrench_multiplication2 (w_p, w_p_t, lever_arm);
counter = 0; 

cable_length_cell = cell(1,length(coordinate.z));

       %Go through all the coordinate of z-axis, and save them in variable workspace_position 
       for k = 1:length(coordinate.z)
           workspace_position = [coordinate.x coordinate.y coordinate.z(k)]'; %workspace_position in a column vector  

            if w_p == 0 && w_p_t == 0
                wrench = zeros(6,1);
                [stop] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC,b_rot_xy, wrench, limit);
                workspace_logical(k) = ~stop; %write the logical for 1 if stop = 0 (no violation exist)
            else

                for index_wp = 1:size(wrench_mat,2)
                    wrench = wrench_mat(:,index_wp); 
                   [stop,cable_length] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC, b_rot_xy, wrench, limit); 
                   if stop == 1
                       break
                   end

                end

                if stop == 0 
                   counter = counter +1; 
                   cable_length_cell{k} = cable_length;
                   workspace_logical(k) = true; %write the logical for 1 if stop = 0 (no violation exist)
                else
                    workspace_logical(k) = false; %write the logical for 0 if stop = 1 (violation exist)
                end

            end
       end

       cable_length_mat = cat(1,cable_length_cell{:});

end

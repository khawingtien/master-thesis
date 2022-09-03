%% Function Arbeitsraum
function [workspace_logical,  b_rot_xy, POI_rot] = Arbeitsraum_khaw(a, b, f_min, f_max,noC, rotation_axis, rot_angle, w_p, w_p_t, workspace_logical, coordinate, limit)

%Define Point of Interest (POI) 
POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)
lever_arm = POI_offset(3);
%Define rotation axis 
R = axang2rotm([rotation_axis, deg2rad(rot_angle)]); 

b_rot_xy = R *b; %Rotation of endeffector 

POI_rot = R *POI_offset; %Rotation of point of interest (the end of endeffector) 

% middle_rod = R *([-POI_offset, POI_offset]); 

%call the function which calculate the wrench direction coordinate 
wrench_mat = wrench_multiplication2 (w_p, w_p_t, lever_arm);

       %Go through all the coordinate of z-axis, and save them in variable workspace_position 
       for k = 1:length(coordinate.z)
           workspace_position = [coordinate.x coordinate.y coordinate.z(k)]'; %workspace_position in a column vector  

            if w_p == 0 && w_p_t == 0
                wrench = zeros(6,1);
                [stop] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC,b_rot_xy, wrench, limit); %hier erstmal nur stop von Interesse tbd
                workspace_logical(1,1,k) = ~stop; %write the logical for 1 if stop = 0 (no violation exist)
            else
                 check_wp_log = zeros(1,26);
                for index_wp = 1:length(wrench_mat)
                    wrench = wrench_mat(:,index_wp); 
                   [stop] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC, b_rot_xy, wrench, limit); 
                   check_wp_log(index_wp) = stop; %as soon as a point does not fulfill the wrench, stop the process

                   if stop == 1
                       break
                   end
                end

                    if any(check_wp_log)  %Determine if any array elements are nonzero or true 
                        workspace_logical(1,1,k) = false; %write the logical for 0 if stop = 1 (violation exist)
                    else 
                        workspace_logical(1,1,k) = true; %write the logical for 1 if stop = 0 (no violation exist)
                    end

            end
       end

     
end

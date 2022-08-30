%% Function Arbeitsraum
function [workspace_logical,  b_rot_xy, POI_rot] = Arbeitsraum_khaw(a, b, f_min, f_max,noC, rot_axis, rot_angle, w_p, w_p_t, workspace_logical, coordinate, limit)
counter = 1; %predefine counter = 1

%Define Point of Interest (POI) 
POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)

%Define rotation axis 
R = axang2rotm([rot_axis, deg2rad(rot_angle)]); 

b_rot_xy = R*b; %Rotation of endeffector 

POI_rot = R *POI_offset; %Rotation of point of interest (the end of endeffector) 

% middle_rod = R *([-POI_offset, POI_offset]); 

[wrench] = wrench_calculation_khaw(POI_rot,w_p,w_p_t,R);

%Go through all the coordinate of z-axis, and save them in variable workspace_position 
           for k = 1:length(coordinate.z)
               workspace_position = [coordinate.x coordinate.y coordinate.z(k)]'; %workspace_position in a column vector  

                %Calculate the force distribution at this position         
                [stop] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC,b_rot_xy, wrench,  limit); %hier erstmal nur stop von Interesse tbd
                counter = counter + 1; %no semicolon, to show the current progression during debugging
                workspace_logical(1,1,k) = ~stop; %write the logical for 1 if stop = 0 (no violation exist) & 0 for stop = 1 (violation exist)
           end
     
end

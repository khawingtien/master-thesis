%% Function Arbeitsraum
function [workspace_logical,  b_rot_xy, POI_rot] = Arbeitsraum_khaw(a, b, f_min, f_max,noC, rotation_axis, rot_angle, w_p, w_p_t, workspace_logical, coordinate, limit)
counter = 1; %predefine counter = 1

%Define Point of Interest (POI) 
POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)

%Define rotation axis 
R = axang2rotm([rotation_axis, deg2rad(rot_angle)]); 

b_rot_xy = R *b; %Rotation of endeffector 

POI_rot = R *POI_offset; %Rotation of point of interest (the end of endeffector) 

% middle_rod = R *([-POI_offset, POI_offset]); 

%% Define the step size of rotation axis for wrench (adjust step size) 
wp_angles_z = 0:45:360; %rotation axis for wp in step size 45° 
wp_rot_angles_3Daxis = 0:10:30; %positive %C-bogen
wp_rot_axis = zeros(length(wp_angles_z),3); %preallocating for speed

check_log_angles = zeros(length(wp_angles_z),1);
check_log_3Daxis = zeros(length(wp_angles_z),length(wp_rot_angles_3Daxis));

% Calculate the rotation axis around z-axis  [3x3 matrix]
for z = 1 : length(wp_angles_z)
    wp_rotation_array_z = [0 0 1 deg2rad(wp_angles_z(z))]; %rotation only at z-axis 
    wp_rot_mat_z = axang2rotm(wp_rotation_array_z); %create 3x3 rotation matrix at z_axis 
    wp_rot_axis (z,:) = [1 0 0] * wp_rot_mat_z; %rotation axis between x-axis and y-axis (start from coordinate 1 0 0)
end

       %Go through all the coordinate of z-axis, and save them in variable workspace_position 
       for k = 1:length(coordinate.z)
           workspace_position = [coordinate.x coordinate.y coordinate.z(k)]'; %workspace_position in a column vector  

            if w_p == 0
                wrench = zeros(6,1);
                [stop] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC,b_rot_xy, wrench, limit); %hier erstmal nur stop von Interesse tbd
                workspace_logical(1,1,k) = ~stop; %write the logical for 1 if stop = 0 (no violation exist)
            else
                %loop for wrench axis 
                for counter_3Daxis = 1 : length(wp_rot_angles_3Daxis) %C-Bogen 0:30°
                    rot_angle_wp = wp_rot_angles_3Daxis(counter_3Daxis); 
                
                    for counter_angles_z = 1: length(wp_angles_z) %rotate at xy-axis (every 45°) to 360° 
                        rot_axis_wp = wp_rot_axis(counter_angles_z,:); %rotation axis for each rotation at xy-axis 
                
                        R_wp = axang2rotm([rot_axis_wp, deg2rad(rot_angle_wp)]); %rotation of wrench 
                        [wrench] = wrench_multiplication(R_wp, w_p, w_p_t,POI_rot);
    
   
                        %Calculate the force distribution at this position         
                        [stop] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC,b_rot_xy, wrench, limit); %hier erstmal nur stop von Interesse tbd
                        counter = counter + 1; %no semicolon, to show the current progression during debugging
                        check_log_angles(counter_angles_z,1) = stop;
                    end
                
                check_log_3Daxis(:,counter_3Daxis) = check_log_angles;
                end
    
                if all(all(check_log_3Daxis)) %Determine if all array elements are nonzero or true 
                    workspace_logical(1,1,k) = false; %write the logical for 0 if stop = 1 (violation exist)
                else 
                     workspace_logical(1,1,k) = true; %write the logical for 1 if stop = 0 (no violation exist)
                end
            end
       end



     
end

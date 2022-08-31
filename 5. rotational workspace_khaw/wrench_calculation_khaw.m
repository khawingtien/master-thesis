function [wrench] = wrench_calculation_khaw(w_p,w_p_t,POI_rot)
%calculation of rotation axis for wrench 

%% Define the step size of rotation axis for wrench 
wp_angles_z = 0:45:360; %rotation axis for wp in step size 45° 
wp_rot_angles_3Daxis = 0:5:30; %positive %C-bogen
wp_rot_axis = zeros(length(wp_angles_z),3); %preallocating for speed
wrench_mat = [];
wrench_mat_total = [];

% Calculate the rotation axis around z-axis  [3x3 matrix]
for z = 1 : length(wp_angles_z)
    wp_rotation_array_z = [0 0 1 deg2rad(wp_angles_z(z))]; %rotation only at z-axis 
    wp_rot_mat_z = axang2rotm(wp_rotation_array_z); %create 3x3 rotation matrix at z_axis 
    wp_rot_axis (z,:) = [1 0 0] * wp_rot_mat_z; %rotation axis between x-axis and y-axis (start from coordinate 1 0 0)
end


for counter_3Daxis = 1 : length(wp_rot_angles_3Daxis) %C-Bogen 0:30°
    rot_angle_wp = wp_rot_angles_3Daxis(counter_3Daxis); 

    for counter_angles_z = 1: length(wp_angles_z) %rotate at xy-axis (every 45°) 
        rot_axis_wp = wp_rot_axis(counter_angles_z,:); %rotation axis for each rotation at xy-axis 

        R_wp = axang2rotm([rot_axis_wp, deg2rad(rot_angle_wp)]); %rotation of wrench 
       [wrench] = wrench_multiplication(R_wp, w_p, w_p_t);
       wrench_mat = [wrench_mat wrench];
    end
    
wrench_mat_total = [wrench_mat_total wrench_mat];
end


end
function [wp_mat] = wrench_multiplication2 (w_p)

w_p=5;
wrench = [0 0 w_p]';

%define rotation step size here 
wp_angles_z = 0:45:315; %rotation axis for wp in step size 45° until 135° 
wp_rot_angles_3Daxis = 45:45:135; %positive C-Bogen only from 45° to 135° 

wp_rot_axis = zeros(length(wp_angles_z),3); %preallocating for speed
wp_mat = zeros(3,length(wp_angles_z)*length(wp_rot_angles_3Daxis)+2); %total of 24loops only, but add 0° ad 180° manually at first and last vector.
counter= 1; %so the first value start with 2 (TWO)!

% Calculate the rotation axis around z-axis  [3x3 matrix]
for z = 1 : length(wp_angles_z)
    wp_rotation_array_z = [0 0 1 deg2rad(wp_angles_z(z))]; %rotation only at z-axis 
    wp_rot_mat_z = axang2rotm(wp_rotation_array_z); %create 3x3 rotation matrix at z_axis 
    wp_rot_axis (z,:) = [1 0 0] * wp_rot_mat_z; %rotation axis between x-axis and y-axis (start from coordinate 1 0 0)
end

% Calculate the wrench matrix for 3D shape
for counter_3Daxis = 1 : length(wp_rot_angles_3Daxis) %C-Bogen 0:45:180° (but only start from 45 to 135°) 
rot_angle_wp = wp_rot_angles_3Daxis(counter_3Daxis); 
    
    for counter_angles_z = 1: length(wp_angles_z) %rotate at xy-axis (every 45°) to 360° 
        counter = counter+1;
        rot_axis_wp = wp_rot_axis(counter_angles_z,:); %rotation axis for each rotation at xy-axis 
        R_wp = axang2rotm([rot_axis_wp, deg2rad(rot_angle_wp)]); %rotation of wrench 
        wrench_vec  = R_wp * wrench; %wrench vector 
        wp_mat(:,counter) = wrench_vec;
    end
end

wp_mat(:,1)=[0 0 w_p];
wp_mat(:,end)=[0 0 -w_p];

%% for visualisation 
% figure
% plot3([wp_mat(1,:)],[wp_mat(2,:)],[wp_mat(3,:)],'.g')  
% daspect([1 1 1])

end
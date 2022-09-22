% function [wrench] = wrench_multiplication2 (w_p, w_p_t, lever_arm)
% 
w_p=5;
w_p_t=5;
lever_arm= 0.3;

unit_vec = [0 0 1]'; %starting point for wrench 

%define rotation step size here 
wp_angles_z = 0:45:315;
wp_rot_angles_3Daxis = 45:45:135; %positive C-Bogen only from 45° to 135° (only 45 to 135° because 0° and 180° has the same repeated point for 8times)
                                  %only from 0 to 180° because the rotation axis already cover the 180° to 360°(opposite of it, so just need to do one of it) 

wp_rot_axis = zeros(length(wp_angles_z),3); %preallocating for speed
sphere_mat = zeros(3,length(wp_angles_z)*length(wp_rot_angles_3Daxis)+2); %total of 24loops only, but add 0° ad 180° manually at first and last vector.
counter= 0; %so the first value start with 2 (TWO)!

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
        counter = counter + 1;
        rot_axis_wp = wp_rot_axis(counter_angles_z,:); %rotation axis for each rotation at xy-axis 
        R_wp = axang2rotm([rot_axis_wp, deg2rad(rot_angle_wp)]); %rotation of wrench 
        sphere_vec  = R_wp * unit_vec; %wrench vector 
        sphere_mat(:,counter) = round(sphere_vec,14);
    end
end

% sphere_mat(:,end)=[0 0 -1]; %Main Problem here
% sphere_mat(:,end-1)=[0 0 -1]; %Main Problem here

wrench_f= sphere_mat* w_p; %external force in N
wrench_t= sphere_mat* w_p_t* -lever_arm; %ecternal torque in Nmm
wrench = [wrench_f;wrench_t];
wrench= round(wrench, 14); %for better overview 
%% for visualisation 
figure %BECAREFUL when turn on this !!!
plot3([sphere_mat(1,:)],[sphere_mat(2,:)],[sphere_mat(3,:)],'.g')
hold on
plot3([wrench_f(1,:)],[wrench_f(2,:)],[wrench_f(3,:)],'.b') 
plot3([wrench_t(1,:)],[wrench_t(2,:)],[wrench_t(3,:)],'.r')  
daspect([1 1 1])

% end
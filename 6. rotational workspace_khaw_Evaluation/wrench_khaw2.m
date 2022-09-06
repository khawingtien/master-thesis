function [wrench_p_f, level_arm_mat] = wrench_khaw2(b, w_p_x,w_p_t,rotation_matrix_wp,f_direction,POI_rot)
%wrench berechnen unter Berücksichtigung von Rotationen definiert in rotation_w_p_x

%define the level arm for torque 
level_arm_mat = POI_rot; %level arm in coordinate form 
switch f_direction
    case "x"
    wrench_p = [w_p_x; 0; 0]; %for f_x at first position of wrench, f_y = 0 cause the rotation already cover the f-y position 
    wrench_p = rotation_matrix.wpy  * wrench_p; %wrench in x direction rotated around y-axis (IMPORTANT)
    wrench_p_t0 = [w_p_t; 0 ;0]; %See Eq 3.5 Pott Book (w_p composed from applied force f_p and applied torque t_p)
    wrench_p_t = cross((rotation_matrix.wpy * wrench_p_t0) , level_arm_mat); %torque (cross product of wrench and level arm) 
    wrench_p_t2 = rotation_matrix.wpx  * cross(wrench_p_t0,level_arm_mat); %rotation matrix multiply with wrench_torque_x vector 

    case "y"
    wrench_p = [0; w_p_x; 0];
    wrench_p = rotation_matrix.wpx  * wrench_p; %wrench in y direction rotated around x-axis (IMPORTANT)
    wrench_p_t0 = [0; w_p_t; 0]; %Kraft  = 5N
    wrench_p_t = cross(rotation_matrix.wpx * wrench_p_t0,level_arm_mat);
%     wrench_p_t = rotation_matrix.wpy   * cross(wrench_p_t0,level_arm_mat);  %rotation matrix multiply with wrench_torque_y vector 

end

%calculation of wrench with torque 
wrench_p_f = [wrench_p; wrench_p_t]; %force_x; force_y; force_z; torque_x; torque_y; torque_z

end
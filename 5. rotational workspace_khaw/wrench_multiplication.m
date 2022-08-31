function [wrench] = wrench_multiplication(R_wp, w_p, w_p_t)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%calculation of force in x,y,z
level_arm = R_wp * [0 0 -300]';
wrench_p = [w_p; w_p; w_p]; %for f_x at first position of wrench, f_y = 0 cause the rotation already cover the f-y position 
wrench_p = R_wp  * wrench_p;

%calculation of torque in x,y,z
wrench_p_t = [w_p_t; w_p_t; w_p_t]; %See Eq 3.5 Pott Book (w_p composed from applied force f_p and applied torque t_p)
wrench_p_t = cross((R_wp * wrench_p_t) , level_arm); %torque (cross product of wrench and level arm) 

wrench = [wrench_p; wrench_p_t]; %[w_x w_y w_z torque_x torque_y torque_z]

end
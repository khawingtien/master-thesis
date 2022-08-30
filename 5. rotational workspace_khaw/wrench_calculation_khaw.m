function [wrench] = wrench_calculation_khaw(POI_rot,w_p,w_p_t,R)

%calculation of force in x,y,z
level_arm = POI_rot;
% level_arm = [0 0 -600]
wrench_p = [w_p; w_p; w_p]; %for f_x at first position of wrench, f_y = 0 cause the rotation already cover the f-y position 
wrench_p = R  * wrench_p;

%calculation of torque in x,y,z
wrench_p_t = [w_p_t; w_p_t; w_p_t]; %See Eq 3.5 Pott Book (w_p composed from applied force f_p and applied torque t_p)
wrench_p_t = cross((R * wrench_p_t) , level_arm); %torque (cross product of wrench and level arm) 

wrench = [wrench_p; wrench_p_t]; %[w_x w_y w_z torque_x torque_y torque_z]

end
function [wrench_p_f] = wrench_khaw(w_p_x,w_p_t,rotation_matrix)
%wrench berechnen unter Berücksichtigung von Rotationen definiert in rotation_w_p_x
 wrench_p.x = [w_p_x; 0; 0]; %for f_x at first position of wrench, f_y = 0 cause the rotation already cover the f-y position 
wrench_p.y = [0; w_p_x; 0];
 wrench_p.x = rotation_matrix.wpy  * wrench_p.x; %wrench in x direction rotated around y-axis (IMPORTANT)
wrench_p.y = rotation_matrix.wpx  * wrench_p.y; %wrench in y direction rotated around x-axis (IMPORTANT)

%calculation of wrench with torque
 wrench_p_t.x = [w_p_t; 0;0]; % Then the third value is torque_x, See Eq 3.5 Pott Book (w_p composed from applied force f_p and applied torque t_p)
wrench_p_t.y = [0; w_p_t; 0]; %torque_y = 0
 wrench_p_t.x = rotation_matrix.wpx  * wrench_p_t.x; %rotation matrix multiply with wrench_torque_x vector 
wrench_p_t.y = rotation_matrix.wpy  * wrench_p_t.y;  %rotation matrix multiply with wrench_torque_y vector 

wrench_p_f.fx = [wrench_p.x; wrench_p_t.x]; %% wrench (f_x) rotated around y-axis
wrench_p_f.fy = [wrench_p.y; wrench_p_t.y]; %% wrench (f_y) rotated around x-axis

end
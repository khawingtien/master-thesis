function [I_vv,Volume_ws,Volume_frame] = evaluation_volume(vol_results_remove_OutL,ax_value,ay_value, b_value)
%The index of volume Ivv evaluates the volume of the workspace relatively to the dimensions of the whole robotic structure.

% hcc = 200; %height of cylinder in mm 
% Dcc = 300; %Diameter of cylinder in mm 

L = ax_value; %in mm
W = ay_value; %in mm %because its symmetry 
rod_length = b_value; %in mm


Volume_ws = vol_results_remove_OutL; %in mm3
Volume_frame = L * W* rod_length; 

I_vv =  Volume_ws / Volume_frame; %dimensionless index 

% Volume_cylinder = (pi* hcc* Dcc^2)/4;

end
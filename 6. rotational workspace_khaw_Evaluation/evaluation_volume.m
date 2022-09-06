function [I_vv] = evaluation_volume(workspace_trans_mat,grid_delta)
%The index of volume Ivv evaluates the volume of the workspace relatively to the dimensions of the whole robotic structure.

p = length(workspace_trans_mat);
delta_x = grid_delta;
delta_y = grid_delta;
delta_z = grid_delta;
hcc = 200; %height of cylinder in mm 
Dcc = 300; %Diameter of cylinder in mm 


%   Detailed explanation goes here
I_vv =  (p* delta_x* delta_y* delta_z)/ (pi* hcc* Dcc^2 /4); %dimensionless index 
end
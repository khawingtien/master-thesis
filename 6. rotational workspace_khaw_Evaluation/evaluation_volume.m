function [I_vv,Volume_ws_m3] = evaluation_volume(workspace_trans_mat,grid_delta,ax_value,ay_value, b_value)
%The index of volume Ivv evaluates the volume of the workspace relatively to the dimensions of the whole robotic structure.

p = length(workspace_trans_mat);
delta_x = grid_delta;
delta_y = grid_delta;
delta_z = grid_delta;
hcc = 200; %height of cylinder in mm 
Dcc = 300; %Diameter of cylinder in mm 

L = ax_value; %in mm
W = ay_value; %in mm %because its symmetry 
rod_length = b_value; %in mm


% Volume_ws = (p* delta_x* delta_y* delta_z); %in mm3 %t

% Volume_ws_m3 = Volume_ws*1e-9; %in m3

%%Plot Boundary in 3D
[k,Volume_ws] = boundary(workspace_trans_mat, 1); %vol in mm 
trisurf(k,workspace_trans_mat(:,1),workspace_trans_mat(:,2),workspace_trans_mat(:,3),'FaceColor','cyan','FaceAlpha',0.2)

Volume_frame = L * W* rod_length; 

I_vv =  Volume_ws / Volume_frame; %dimensionless index 

% Volume_cylinder = (pi* hcc* Dcc^2)/4;

end
function [workspace_pointwise_trans] = ws_translation_khaw(workspace_logical,coordinate,POI_rot)
%Plot the feasible workspace here

% Detailed explanation goes here
sz = [length(coordinate.x), length(coordinate.y), length(coordinate.z)]; %define the size of matrix for command ind2sub  
index_ws_point = find(workspace_logical == 1);
workspace_pointwise = zeros(length(index_ws_point),3);

% if isempty(index_ws_point)
%     workspace_pointwise = [0 0 -200];  
% else
     for j = 1 : length(index_ws_point)
            [id_x, id_y, id_z] = ind2sub(sz, index_ws_point(j)); %Convert linear indices to subscripts with dimension of grid_size(67)
            workspace_pointwise(j, 1) = coordinate.x(id_x); %select x-coordinate from workspace 
            workspace_pointwise(j, 2) = coordinate.y(id_y); %select y-coordinate from workspace 
            workspace_pointwise(j, 3) = coordinate.z(id_z); %select z-coordinate from workspace 
     end
     
% end

workspace_pointwise_trans = workspace_pointwise + POI_rot'; %translation of the results to POI
end

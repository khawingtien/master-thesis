function [workspace_trans_remove_OutL,vol_results_remove_OutL,cable_length_mat_cell_mat,w_p] = maincode_khaw(a,b,noC)

% close all
% clear
% clc

maincode_timer = tic; %start Stopwatch timer
 

% Define max and min of grid in z-direction
grid.z_max = 50;
grid.z_min = -650;

%Definiere Grid    
grid_n = 40;  %Anzahl der Unterteilungen in X-Richtung
grid_delta = (grid.z_max - grid.z_min) / grid_n;  %discretization steps in x-direction in mm %Gitterabstand von X-Richtung (Y- & Z-Richtung auch in diesem Abstand)



%% Definition of rotation axis 
% Define rotation value 
rotation_angles_z = 0:5:355; %rotation pro quadrant  (0:90)
rotation_angles_3Daxis = 0:5:80; %positive %C-bogen 

% Preallocationg for speed 
rot_axis = zeros(length(rotation_angles_z),3);

%Define rotation axis matrix 
for z = 1 : length(rotation_angles_z)
    rotation_array_z = [0 0 1 deg2rad(rotation_angles_z(z))]; %rotation only at z-axis 
    rot_mat_z = axang2rotm(rotation_array_z); %create 3x3 rotation matrix at z_axis 
    rot_axis (z,:) = [1 0 0] * rot_mat_z; %rotation axis between x-axis and y-axis (start from coordinate 1 0 0)
end

%% Definiere zu untersuchende Lasten in bestimmte Raumrichtungen definiert durch rotation_w_p
w_p = 5; %N wrench in x-y-z direction 
w_p_t = 5; %N wrench in Torque in x-y-z direction (Feedback Kraft in Rotation)

%% Parameter zur Arbeitsraum Berechnung
Mn = 183; %Nenndrehmoment in unit mNm for Motor 
L_winde = 4.5; %mm %Stand:21.09.2022
f_min = 5;
f_max = Mn/L_winde; 
limit.lower = (1/2 * (f_max - f_min)) ; %upper limit for improve closed-form solution (eq. 3.6 Pott book)
limit.upper = (1/2 * sqrt(noC) * (f_max - f_min)); %lower limit for improved closed form (eq. 3.6 Pott book)

coordinate.x = 0; %step size in x-direction
coordinate.y = 0; %step size in y-direction
coordinate.z = (grid.z_min : grid_delta: grid.z_max)'; %step size in z-direction

workspace_logical = ~ones(1,length(coordinate.z)); %preallocating the variable for speed (logical)

total_counter = 0;
workspace_cell= cell(length(rotation_angles_3Daxis)*length(rotation_angles_z),1);
cable_length_mat_cell = cell(length(rotation_angles_3Daxis)*length(rotation_angles_z),1);
% ws_logical_cell = cell (length(rotation_angles_z),length(rotation_angles_3Daxis));

%% Maincode 

for counter_angles_z = 1: length(rotation_angles_z) %0:360° rotate at xy-axis (4 Quadrant) 
        rotation_axis = rot_axis(counter_angles_z,:); %rotation axis for each rotation at xy-axis 

    for counter_3Daxis = 1 : length(rotation_angles_3Daxis) %C-Bogen 0:30°

        if counter_3Daxis==1 && counter_angles_z~=1 %only to rotate the 0° for ONCE, else same point would be repeated for 36 times. 
            continue
        end
        
        rot_angle = rotation_angles_3Daxis(counter_3Daxis); 

        %calculate the points that are within the workspace 
        [workspace_logical,  ~, POI_rot, cable_length_mat] = Arbeitsraum_khaw(a, b, f_min, f_max, noC,  rotation_axis, rot_angle, w_p, w_p_t, workspace_logical, coordinate, limit);
%             ws_log.POI=POI_rot;
%             ws_log.logical=workspace_logical;
%             ws_logical_cell{counter_angles_z, counter_3Daxis}=ws_log;

        %convert the workspace point to the POI (at the end of endeffector)
        [workspace_pointwise_trans] = ws_translation_khaw(workspace_logical,coordinate,POI_rot);
        total_counter = total_counter+1;
        workspace_cell{total_counter} = workspace_pointwise_trans; %save the ws points with translation in a cell array 
        cable_length_mat_cell{total_counter} = cable_length_mat;
    end
end
    
        workspace_trans_mat = cat(1,workspace_cell{:}); %concatenate array into matrix 
        cable_length_mat_cell_mat = cat(1,cable_length_mat_cell{:});
    
        %Remove Outliers in plot 
        workspace_trans_remove_OutL = remove_outliers(workspace_trans_mat);
    

        % plot the workspace WITH Outliers (for comparison purpose only) 
%         [vol_results] = ws_plot_khaw(workspace_trans_mat,a,b,w_p,noC, w_p_t);

        % plot the workspace WITHOUT Outliers & calculation
%         [vol_results_remove_OutL] = ws_plot_khaw(workspace_trans_remove_OutL,a,b,noC); 
        [~,vol_results_remove_OutL] = boundary(workspace_trans_remove_OutL, 1); %in mm3 %only calculation, without plot. 
        
        %plot the convexhull area and Volume of convex hull 
%         [convexhull_volume, ~,indices] = convexhull_khaw(workspace_trans_mat);



toc(maincode_timer)


end

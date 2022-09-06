% close all
clear
clc
tic %start Stopwatch timer
figure 

%% Frame parameter 
% ax = 0.230; %in m 
% ay = 0.230; %in m 
% az = 0.06; %in m 

[a_cell] = SetupParameter();
% a = a_cell{1,1};

%% Endeffector parameter 
b_cell = endeffektor2();

%% Standardparameter
noC = 8;
    
% Define max and min of grid in z-direction
grid.z_max = 50;
grid.z_min = -650;

%Definiere Grid    
grid_n = 40;  %Anzahl der Unterteilungen in X-Richtung
grid_delta = (grid.z_max - grid.z_min) / grid_n;  %discretization steps in x-direction in mm %Gitterabstand von X-Richtung (Y- & Z-Richtung auch in diesem Abstand)



%% Definition of rotation axis 
% Define rotation value 
rotation_angles_z = 0:10:350; %rotation pro quadrant  (0:90)
rotation_angles_3Daxis = 0:5:30; %positive %C-bogen 

% Preallocationg for speed 
rotation_array_xy = zeros(length(rotation_angles_z),4); 
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
f_min = 5;
f_max = 36; % fmax berechnet: 2* 183 / 10 = 36, 6 %Motor 
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

%% Definition for Index evaluation
I_vv_cell = cell(length(b_cell),1);
I_vv_cell_ax = cell(length(a_cell),1);
rod_length = zeros(length(b_cell),1);
ax_length = zeros(length(a_cell),1);

%% Maincode
for counter_ax = 1 : length(a_cell)
        a = cell2mat(a_cell(counter_ax,1));
        ax_length(counter_ax,1) = a(1,1)*2; %
%         total_counter = 0;

    for counter_b = 1 : length(b_cell)
        b = cell2mat(b_cell(counter_b,1));
        rod_length(counter_b,1) = b(3,1)*2; %rod length is the b_z parameter multiply by TWO
        total_counter = 0; 

        for counter_angles_z = 1: length(rotation_angles_z) %0:360° rotate at xy-axis (4 Quadrant) 
                rotation_axis = rot_axis(counter_angles_z,:); %rotation axis for each rotation at xy-axis 
    
            for counter_3Daxis = 1 : length(rotation_angles_3Daxis) %C-Bogen 0:30°
                rot_angle = rotation_angles_3Daxis(counter_3Daxis); 
        
                %calculate the points that are within the workspace 
                [workspace_logical,  b_rot_xz, POI_rot, cable_length_mat] = Arbeitsraum_khaw(a, b, f_min, f_max, noC,  rotation_axis, rot_angle, w_p, w_p_t, workspace_logical, coordinate, limit);
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
    
        
        % plot the workspace 
        [w_p] = ws_plot_khaw(workspace_trans_mat,a,b,w_p,noC, w_p_t);
        
        % plot the convexhull area and Volume of convex hull 
        [convexhull_volume, ~,indices] = convexhull_khaw(workspace_trans_mat);
        
        %Evaluation of Index of Volume
        [I_vv] = evaluation_volume(workspace_trans_mat,grid_delta); 
%         I_vv_cell{counter_b} = I_vv;
          I_vv_cell{counter_ax} = I_vv;
    end
     
end
       

I_vv_mat = cat(1,I_vv_cell{:});
% I_vv_mat_ax = cat(1,I_vv_cell_ax{:});

% %function to plot the evaluation of Index 
% evaluation_plot(I_vv_mat,rod_length); %Case 1 
 evaluation_plot(I_vv_mat, ax_length); %Case 2 



toc




clear
clc
% close all
evaluation_timer=tic;

%%Define Parameter here 
noC = 8;

%% Define iteration for z-axis (Translation in z-direction)
%Define max and min of grid in z-direction
grid.z_max = 50;
grid.z_min = -600;

%Define Grid    
grid_n = 40;  %stepsize in x- and y-direction 
grid_delta = (grid.z_max - grid.z_min) / grid_n;  %discretization steps in x-direction in mm %Gitterabstand von X-Richtung (Y- & Z-Richtung auch in diesem Abstand)

coordinate.x = 0; %step size in x-direction
coordinate.y = 0; %step size in y-direction
coordinate.z = (grid.z_min : grid_delta: grid.z_max)'; %step size in z-direction

%% Define wrench (either 5N or 0N = no wrench) 
w_p = 5; %N wrench in x-y-z direction 
w_p_t = 5; %N wrench in Torque in x-y-z direction (Feedback Kraft in Rotation)

%% Parameter for workspace calculation
Mn = 183; %Nenndrehmoment in unit mNm for Motor 
L_winde = 4.5; %mm %Stand:21.09.2022
f_min = 5; %Newton to prevent the cable from sagging  
f_max = Mn/L_winde; 
limit.lower = (1/2 * (f_max - f_min)) ; %upper limit for improve closed-form solution (eq. 3.6 Pott book)
limit.upper = (1/2 * sqrt(noC) * (f_max - f_min)); %lower limit for improved closed form (eq. 3.6 Pott book)

%% Calculation for average feasible force 
f_M = ones(noC,1); %preallocating for speed
f_M = f_M .* ((f_min + f_max) / 2); % f_M = average feasible force (below Eq 3.53 Pott Book)

%% Definition of rotation axis 
% Define rotation value 
rotation_angles_z = 0:5:355; %rotation around z-axis 
rotation_angles_3Daxis = 0:5:80; %positive %C-bogen 

rot_axis = zeros(length(rotation_angles_z),3); %Preallocationg for speed 

%Define rotation axis matrix 
for z = 1 : length(rotation_angles_z)
    rotation_array_z = [0 0 1 deg2rad(rotation_angles_z(z))]; %rotation only at z-axis 
    rot_mat_z = axang2rotm(rotation_array_z); %create 3x3 rotation matrix at z_axis 
    rot_axis (z,:) = [1 0 0] * rot_mat_z; %rotation axis between x-axis and y-axis (start from coordinate 1 0 0)
end



%% Definition for frame & rod length 
%choose one to set a changing variable
ax_range = 300:50:600; %in m
az_range = 0:50:300; %in m
b_range = 300:50:600; %in m

%the rest of the parameter will be fixed 
ax_standard = 510; %in m (INPUT: total of ax) (predefined: always ax = ay) 
ay_standard = 510; %in m (INPUT: total of ay) 
az_standard = 50; %in m (INPUT: total of az)
 b_standard = 600; %in m (INPUT: total rod lenght)
 
Parameter = "standard"; %Define changing parameter here 

switch Parameter
   case "standard"
        group = 0;
        [a_cell] = SetupParameter(ax_standard,ay_standard,az_standard);
        b_cell =  endeffector_parameter(b_standard);
        x_label = sprintf("standard [mm]");
        loop_length = length(b_cell);
        zeros(length(b_cell),1);

    case "ax"
        group = 1;
        [a_cell] = SetupParameter(ax_range,ay_standard,az_standard);%variety
        b_cell =  endeffector_parameter(b_standard);
        x_label = sprintf("ax length [mm]");
        loop_length = length(a_cell);

    case "az" 
        group = 2;
        [a_cell] = SetupParameter(ax_standard,ay_standard,az_range);%variety
        b_cell =  endeffector_parameter(b_standard);
        x_label = sprintf("az length [mm]");
        loop_length = length(a_cell);
        
    case "b"
        group = 3;
        [a_cell] = SetupParameter(ax_standard,ay_standard,az_standard);
        b_cell =  endeffector_parameter(b_range); %variety
        x_label = sprintf("rod length [mm]");
        loop_length = length(b_cell);

end

b = cell2mat(b_cell(1,1)); %standard parameter (it would be overwriten if its not standard)
a = cell2mat(a_cell(1,1)); %standard parameter (it would be overwriten if its not standard)

I_vv_cell = cell(max(length(a_cell), length(b_cell)),1); %predefine for speed 
Volume_ws_cell = cell(max(length(a_cell), length(b_cell)),1); %predefine for speed 
Volume_frame_cell = cell(max(length(a_cell), length(b_cell)),1); %predefine for speed 
Percentage_cell = cell(max(length(a_cell), length(b_cell)),1); %predefine for speed 
plot_x_axis = zeros(max(length(a_cell), length(b_cell)), 1); %predefine for speed 


%% Define Point of Interest (POI) 
POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)
lever_arm = POI_offset(3); %only half of rod length 

%% Define the points to test for wrench 
wrench_mat = wrench_multiplication2 (w_p, w_p_t, lever_arm);

%% Start the maincode
for counter = 1 : loop_length
    switch Parameter     
        case "standard"
            a = cell2mat(a_cell(1,1));
            ax_value = a(1,1)*2;
            ay_value = a(1,1)*2;
            az_value = a(3,1)*2;
            b_value = b_cell{1,1}(3)*2;
            plot_x_axis(1,1) = a(1,1)*2; %times two bcz only half of the length
            txt = ['ax = ' int2str(ax_value) ' ay = ' int2str(ay_value) ' az = ' int2str(az_value)  ' rod length = ' int2str(b_value) ' [mm]'];

        case "ax"
            a = cell2mat(a_cell(counter,1)); %variety of parameter
            ax_value = a(1,1)*2;
            ay_value = a(2,1)*2;
            az_value = a(3,1)*2;
            b_value = b_cell{1,1}(3)*2;
            plot_x_axis(counter,1) = a(1,1)*2; %times two bcz only half of the length
            txt = ['ay = ' int2str(ay_value) ' az = ' int2str(az_value)  ' rod length = ' int2str(b_value) ' [mm]']; 

        case "az"
            a = cell2mat(a_cell(counter,1)); %variety of parameter
            ax_value = a(1,1)*2; 
            ay_value = a(2,1)*2; 
            az_value = a(3,1)*2; 
            b_value = b_cell{1,1}(3)*2;
            plot_x_axis(counter,1) = a(3,1)*2;  %times two bcz only half of the length
            txt = ['ax = ' int2str(ax_value) ' ay = ' int2str(ay_value), ' rod length = ' int2str(b_value) ' [mm]']; 

        case "b"
            a = cell2mat(a_cell);
            ax_value = a(1,1)*2; %same as ax_standard
            ay_value = a(2,1)*2; %same as ax_standard
            az_value = a(3,1)*2; %same as ax_standard
            b = cell2mat(b_cell(counter,1)); %variety of parameter
            b_value = b(3,1)*2; %times two bcz only half of the length
            plot_x_axis(counter,1) = b_value; 
            txt = ['ax = ' int2str(ax_value) ' ay = ' int2str(ay_value), ' az = ' int2str(az_value) ' [mm]']; 

    end

        total_counter = 0; %reset counter  
        [workspace_trans_remove_OutL,vol_results_remove_OutL,cable_length_mat_cell_mat,w_p] = maincode_khaw(a,b,noC,coordinate,w_p,w_p_t,rotation_angles_z,rotation_angles_3Daxis,rot_axis,f_min,f_max,limit,f_M,POI_offset,wrench_mat);
        
        % 4 specs analysis
        [I_vv,Volume_ws,Volume_frame] = evaluation_volume(vol_results_remove_OutL,ax_value,ay_value,b_value);
        [Percentage,Vol_point_in_ROI]  = Percentage_within_ROI(workspace_trans_remove_OutL,b_value);

        I_vv_cell{counter} = I_vv; 
        Volume_ws_cell{counter} = Volume_ws;
        Volume_frame_cell{counter} = Volume_frame;
        Percentage_cell{counter} = Percentage;  


[vol_results] = ws_plot_winner_khaw(workspace_trans_remove_OutL,a,b,noC,I_vv,Volume_ws*1e-9,Volume_frame*1e-9,Percentage,w_p);


% %save figure automatically
% path = 'D:\Masterarbeit\11_MATLAB_GIT\6. rotational workspace_khaw_Evaluation\figure\20220912';
% filename = "rotational_workspace_%d_%d_%d_%d_%d";
% filename_total = sprintf(filename,group, round(ax_value,0) ,round(ay_value,0), round(az_value,0), round(b_value,0));
% saveas(gcf,fullfile(path,filename_total),'fig');
% close(gcf)

end

I_vv_mat = cat(1,I_vv_cell{:});
Volume_ws_mat = cat(1,Volume_ws_cell{:});
Volume_frame_mat = cat(1,Volume_frame_cell{:});
Percentage_mat = cat(1, Percentage_cell{:});


% evaluation_plot(plot_x_axis,I_vv_mat,Volume_ws_mat,Volume_frame_mat,Percentage_mat,x_label,txt);

%save figure automatically
% path = 'D:\Masterarbeit\11_MATLAB_GIT\6. rotational workspace_khaw_Evaluation\figure\20220912';
% filename = "rotational_workspace_%d_%s";
% filename_total = sprintf(filename,group,x_label);
% saveas(gcf,fullfile(path,filename_total),'fig');
% close(gcf)


disp(strcat("Total execution time: ", num2str(toc(evaluation_timer)), "s"))

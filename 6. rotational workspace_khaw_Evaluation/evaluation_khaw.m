clear
clc
% close all
evaluation_timer=tic;

%% Definition for frame & rod length 
%choose one to set a changing variable
ax_range = 0.3:0.05:0.6; %in m
az_range = 0.0:0.05:0.3; %in m
b_range = 0.3:0.05:0.6; %in m

%the rest of the parameter will be fixed 
ax_standard = 0.5; %in m (INPUT: total of ax) (predefined: always ax = ay) 
ay_standard = 0.5; %in m (INPUT: total of ay) 
az_standard = 0.02; %in m (INPUT: total of az)
 b_standard = 0.4; %in m (INPUT: total rod lenght)
 
Parameter = "az"; %Define changing parameter here 

switch Parameter
   case "standard"
        group = 0;
        [a_cell] = SetupParameter(ax_standard,ay_standard,az_standard);
        b_cell = endeffektor2(b_standard);
        x_label = sprintf("standard [mm]");
        loop_length = length(b_cell);
        zeros(length(b_cell),1);

    case "ax"
        group = 1;
        [a_cell] = SetupParameter(ax_range,ay_standard,az_standard);%variety
        b_cell = endeffektor2(b_standard);
        x_label = sprintf("ax length [mm]");
        loop_length = length(a_cell);

    case "az" %extra 
        group = 2;
        [a_cell] = SetupParameter(ax_standard,ay_standard,az_range);%variety
        b_cell = endeffektor2(b_standard);
        x_label = sprintf("az length [mm]");
        loop_length = length(a_cell);
        
    case "b"
        group = 3;
        [a_cell] = SetupParameter(ax_standard,ay_standard,az_standard);
        b_cell = endeffektor2(b_range); %variety
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
            b_value = b(3,1)*2;
            plot_x_axis(counter,1) = b_value; %times two bcz only half of the length
            txt = ['ax = ' int2str(ax_value) ' ay = ' int2str(ay_value), ' az = ' int2str(az_value) ' [mm]']; 

    end

        total_counter = 0; %reset counter 
        maincode_Khaw_20220824 %call the file 

        [I_vv,Volume_ws,Volume_frame] = evaluation_volume(vol_results_remove_OutL,ax_value,ay_value,b_value);
        [Percentage,Vol_point_in_ROI]  = Percentage_within_ROI(workspace_trans_remove_OutL,b_value);

        I_vv_cell{counter} = I_vv; 
        Volume_ws_cell{counter} = Volume_ws;
        Volume_frame_cell{counter} = Volume_frame;
        Percentage_cell{counter} = Percentage;  

% 
% %save figure automatically
path = 'D:\Masterarbeit\11_MATLAB_GIT\6. rotational workspace_khaw_Evaluation\figure\20220912';
filename = "rotational_workspace_%d_%d_%d_%d_%d";
filename_total = sprintf(filename,group, round(ax_value,0) ,round(ay_value,0), round(az_value,0), round(b_value,0));
saveas(gcf,fullfile(path,filename_total),'fig');
close(gcf)

end

I_vv_mat = cat(1,I_vv_cell{:});
Volume_ws_mat = cat(1,Volume_ws_cell{:});
Volume_frame_mat = cat(1,Volume_frame_cell{:});
Percentage_mat = cat(1, Percentage_cell{:});


 evaluation_plot(plot_x_axis,I_vv_mat,Volume_ws_mat,Volume_frame_mat,Percentage_mat,x_label,txt);

%save figure automatically
path = 'D:\Masterarbeit\11_MATLAB_GIT\6. rotational workspace_khaw_Evaluation\figure\20220912';
filename = "rotational_workspace_%d_%s";
filename_total = sprintf(filename,group,x_label);
saveas(gcf,fullfile(path,filename_total),'fig');
% close(gcf)


disp(strcat("Total execution time: ", num2str(toc(evaluation_timer)), "s"))

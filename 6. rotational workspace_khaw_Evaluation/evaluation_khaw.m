% % function [outputArg1,outputArg2] = evaluation_khaw(inputArg1,inputArg2)
%Evaluation of Ivv here. 

evaluation_timer=tic;


%% Definition for n
ax_range = 0.1:0.1:0.7; %in m
az_range = 0.01:0.05:0.3; %in m
b_range = 0.2:0.3:1.0; %in m

ax_standard = 0.46; %in m (INPUT: total of ax) (predefined: always ax = ay) 
ay_standard = 0.46; 
az_standard = 0.02; %in m (INPUT: total of az)
 b_standard = 0.60; %in m (INPUT: total rod lenght)
 
Parameter = "ax";

switch Parameter
   case "standard"
        [a_cell] = SetupParameter(ax_standard,az_standard);
        b_cell = endeffektor2(b_standard);
        x_label = sprintf("standard [mm]");
        loop_length = length(b_cell);
        zeros(length(b_cell),1);
    case "ax"
        [a_cell] = SetupParameter(ax_range, az_standard);
        b_cell = endeffektor2(b_standard);
        x_label = sprintf("ax length [mm]");
        loop_length = length(a_cell);

    case "az" %extra 
        [a_cell] = SetupParameter(ax_standard, az_range);
        b_cell = endeffektor2(b_standard);
        x_label = sprintf("az length [mm]");
        loop_length = length(a_cell);
        
    case "b"
        [a_cell] = SetupParameter(ax_standard,az_standard);
        b_cell = endeffektor2(b_range);
        x_label = sprintf("rod length [mm]");
        loop_length = length(b_cell);

end

b = cell2mat(b_cell(1,1)); %standard parameter (it would be overwriten if its not standard)
a = cell2mat(a_cell(1,1)); %standard parameter (it would be overwriten if its not standard)

I_vv_cell = cell(max(length(a_cell), length(b_cell)),1); %predefine for speed 
plot_x_axis = zeros(max(length(a_cell), length(b_cell)), 1); %predefine for speed 

for counter = 1 : loop_length
    switch Parameter     
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
            plot_x_axis(counter,1) = a(3,1)*2; 
            txt = ['ax = ' int2str(ax_value) ' ay = ' int2str(ay_value), ' rod length = ' int2str(b_value) ' [mm]']; 

        case "b"
            b = cell2mat(b_cell(counter,1)); %variety of parameter
            plot_x_axis(counter,1) = b(3,1)/2;
    end

        total_counter = 0; %reset counter 
        maincode_Khaw_20220824 %call the file 

        [I_vv,Volume_ws_m3] = evaluation_volume(workspace_trans_mat,grid_delta,ax_value,ay_value, b_value);
        I_vv_cell{counter} = I_vv; 
end

I_vv_mat = cat(1,I_vv_cell{:});

evaluation_plot(plot_x_axis,I_vv_mat,x_label,txt);

disp(strcat("Total execution time: ", num2str(toc(evaluation_timer)), "s"))


%%Plot Boundary in 3D
% [k,vol] = boundary(workspace_trans_mat, 1);
% trisurf(k,workspace_trans_mat(:,1),workspace_trans_mat(:,2),workspace_trans_mat(:,3),'FaceColor','yellow','FaceAlpha',0.1)





% % function [outputArg1,outputArg2] = evaluation_khaw(inputArg1,inputArg2)
%UNTITLED Summary of this function goes here
evaluation_timer=tic;
%   Detailed explanation goes here
%% Frame parameter 

% [a_cell] = SetupParameter();

%% Endeffector parameter 


%% Definition for Index evaluation
ax_range = 0.1:0.1:0.7;
az_range = 0.05:0.1:1;
b_range = 0.2:0.3:2.6;
ax_standard = 0.23;
az_standard = 0.06;
b_standard = 0.6;

Parameter="standard";

switch Parameter
   case "standard"
        [a_cell] = SetupParameter(ax_standard,az_standard);
        b_cell = endeffektor2(b_standard);
        x_label = "rod length";
        loop_length = length(b_cell);
        zeros(length(b_cell),1);
    case "ax"
        [a_cell] = SetupParameter(ax_range, az_standard);
        b_cell = endeffektor2(b_standard);
        x_label = "ax length";
        loop_length = length(a_cell);

    case "az"
        [a_cell] = SetupParameter(ax_standard, az_range);
        b_cell = endeffektor2(b_standard);
        x_label = "az length";
        loop_length = length(a_cell);
        
    case "b"
        [a_cell] = SetupParameter(ax_standard,az_standard);
        b_cell = endeffektor2(b_range);
        x_label = "rod length";
        loop_length = length(b_cell);

end

b = cell2mat(b_cell(1,1)); %standard parameter
a = cell2mat(a_cell(1,1)); %standard parameter

I_vv_cell = cell(max(length(a_cell), length(b_cell)),1);
plot_axis = zeros(max(length(a_cell), length(b_cell)), 1);

for counter = 1 : loop_length
    switch Parameter     
        case "ax"
            a = cell2mat(a_cell(counter,1)); %variety of parameter
            plot_axis(counter,1) = a(1,1)*2;

        case "az"
            a = cell2mat(a_cell(counter,1)); %variety of parameter
            plot_axis(counter,1) = a(3,1)*2;

        case "b"
            b = cell2mat(b_cell(counter,1)); %variety of parameter
            plot_axis(counter,1) = b(3,1)*2;
    end

        total_counter = 0; %reset counter 
        maincode_Khaw_20220824 %call the file 

        [I_vv] = evaluation_volume(workspace_trans_mat,grid_delta);
        I_vv_cell{counter} = I_vv; 
end

 I_vv_mat = cat(1,I_vv_cell{:});

evaluation_plot(I_vv_mat, plot_axis, x_label);

disp(strcat("Total execution time: ", num2str(toc(evaluation_timer)), "s"))

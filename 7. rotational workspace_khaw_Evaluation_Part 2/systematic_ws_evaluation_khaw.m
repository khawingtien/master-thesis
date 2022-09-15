%Systematische Untersuchung von Arbeitsraum 

clc
close all 
clear 

total_time = tic;

range_ax = 300:300:600; %in mm 
range_az = 0:100:300; %in mm 
rod_length = 300:300:600; %in mm 

cell_length = length(range_ax)*length(range_az)*length(rod_length);
workspace_trans_remove_OutL_cell = cell(cell_length,1);
I_vv_cell = cell(cell_length,1);
Volume_ws_cell = cell(cell_length,1);
Volume_frame_cell = cell(cell_length,1);
Percentage_cell = cell(cell_length,1);
index_counter = 0


for index_ax = 1:length(range_ax) %input variable 1 
    ax_value = range_ax(index_ax);
    ay_value = ax_value; 

    for index_az = 1:1:length(range_az) %input variable 2
        az_value = range_az(index_az);

        for index_b = 1:length(rod_length) %input variable 3
            bz_value = rod_length(index_b);

            [a] = Setup_Parameter_khaw(ax_value,ay_value,az_value); %input in mm 
            [b] = Setup_Parameter_Endeffector_khaw(bz_value); %input in mm 

            index_counter = index_counter + 1;
            disp("Calculation "+index_counter+" of "+cell_length)

            total_counter = 0; %reset counter 
%             maincode_Khaw_20220824 %call the file 
            [workspace_trans_remove_OutL,vol_results_remove_OutL] = maincode_khaw(a,b);


            [I_vv,Volume_ws,Volume_frame] = evaluation_volume(vol_results_remove_OutL,ax_value,ay_value,bz_value);
            [Percentage,Vol_point_in_ROI]  = Percentage_within_ROI(workspace_trans_remove_OutL,bz_value);

            %Output Evaluation Parameter  
            workspace_trans_remove_OutL_cell{index_counter,1} = workspace_trans_remove_OutL;
            I_vv_cell{index_counter,1} = I_vv; 
            Volume_ws_cell{index_counter,1} = Volume_ws;
            Volume_frame_cell{index_counter,1} = Volume_frame;
            Percentage_cell{index_counter,1} = Percentage; 

            %save figure automatically
            path = '..\7. rotational workspace_khaw_Evaluation_Part 2\figure\20220915';
            filename = "rotational_workspace_%d_%d_%d_%d_%d_%d";
            filename_total = sprintf(filename,round(index_counter,0),round(index_counter,0), round(ax_value,0) ,round(ay_value,0), round(az_value,0), round(bz_value,0));
            saveas(gcf,fullfile(path,filename_total),'fig');
            close(gcf)

        end
    end
end

% workspace_trans_remove_OutL_mat = cat(1,workspace_trans_remove_OutL_cell{:});
I_vv_mat = cat(1,I_vv_cell{:});
Volume_ws_mat = cat(1,Volume_ws_cell{:});
Volume_frame_mat = cat(1,Volume_frame_cell{:});
Percentage_mat = cat(1, Percentage_cell{:});

[winner_Ivv, Ivv_idx ] = max(I_vv_mat);
[winner_Volume_ws, Volume_ws_idx] = max(Volume_ws_mat);
[winner_Volume_frame, Volume_frame_idx] = max(Volume_frame_mat);
[winner_Percentage, Percentage_idx] = max(Percentage_mat);


ind = [Ivv_idx, Volume_ws_idx, Volume_frame_idx,Percentage_idx];
sz = [length(range_ax) length(range_az) length(rod_length)];
[I1,I2,I3] = ind2sub(sz,ind);

save('Summary report 20220915')

toc(total_time)

function [workspace_trans_remove_OutL,vol_results_remove_OutL,cable_length_mat_cell_mat,w_p] = maincode_khaw(a,b,noC,coordinate,w_p,w_p_t,rotation_angles_z,rotation_angles_3Daxis,rot_axis,f_min,f_max,limit,f_M,POI_offset,wrench_mat)

maincode_timer = tic; %start Stopwatch timer

workspace_logical = ~ones(1,length(coordinate.z)); %preallocating the variable for speed (logical)
workspace_cell= cell(length(rotation_angles_3Daxis)*length(rotation_angles_z),1);
cable_length_mat_cell = cell(length(rotation_angles_3Daxis)*length(rotation_angles_z),1);
total_counter = 0;

%% Mainloop here 
for counter_angles_z = 1: length(rotation_angles_z) %0:360° rotate at xy-axis (4 Quadrant) 
        rotation_axis = rot_axis(counter_angles_z,:); %rotation axis for each rotation at xy-axis 

    for counter_3Daxis = 1 : length(rotation_angles_3Daxis) %C-Bogen 0:30°

        if counter_3Daxis==1 && counter_angles_z~=1 %only to rotate the 0° for ONCE, else same point would be repeated for 36 times. 
            continue
        end
        
        rot_angle = rotation_angles_3Daxis(counter_3Daxis); 
        %Define rotation 
        R = axang2rotm([rotation_axis, deg2rad(rot_angle)]);
        b_rot_xy = R*b; %Rotation of endeffector 
        POI_rot = R*POI_offset; %Rotation of point of interest (the end of endeffector) 

        %calculate the points that are within the workspace 
        [workspace_logical, cable_length_mat] = Arbeitsraum_khaw(a, f_min, f_max, noC,  b_rot_xy, w_p, w_p_t, workspace_logical, coordinate, limit, f_M, wrench_mat);

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

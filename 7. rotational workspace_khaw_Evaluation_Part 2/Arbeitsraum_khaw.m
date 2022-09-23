%% Function Arbeitsraum
function [workspace_logical, cable_length_mat] = Arbeitsraum_khaw(a, f_min, f_max,noC, b_rot_xy, w_p, w_p_t, workspace_logical, coordinate, limit, f_M, wrench_mat)

cable_length_cell = cell(1,length(coordinate.z)); %preallocating for speed

       %Go through all the coordinate of z-axis, and save them in variable workspace_position 
       for k = 1:length(coordinate.z)
           workspace_position = [coordinate.x coordinate.y coordinate.z(k)]'; %workspace_position in a column vector  

            if w_p == 0 && w_p_t == 0
                wrench = zeros(6,1);
                [stop] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC,b_rot_xy, wrench, limit,f_M);
                workspace_logical(k) = ~stop; %write the logical for 1 if stop = 0 (no violation exist)
            else

                for index_wp = 1:size(wrench_mat,2)
                    wrench = wrench_mat(:,index_wp); 
                   [stop,cable_length] = berechnungSeilkraftverteilung_KHAW(workspace_position, a, f_min, f_max,noC, b_rot_xy, wrench, limit,f_M); 
                   if stop == 1
                       break
                   end

                end

                if stop == 0 
                   cable_length_cell{k} = cable_length;
                   workspace_logical(k) = true; %write the logical for 1 if stop = 0 (no violation exist)
                else
                    workspace_logical(k) = false; %write the logical for 0 if stop = 1 (violation exist)
                end

            end
       end

       cable_length_mat = cat(1,cable_length_cell{:});

end

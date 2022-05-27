%% Function Arbeitsraum_Verarbeitung
function [analysis, workspace_logical] = Arbeitsraum_Verarbeitung_KHAW(a, b, f_min, f_max, grid_n, b_name,  w_p, w_p_t, f_g, counter_analysis, rot_name, analysis, workspace, workspace_logical, R, grid_deg)
%% AUSWERTUNG Arbeitsraum
workspace_adapt = workspace_logical; %workspace_logical has the final logic of all wrench-frasible working space in 3D
grid_size = grid_n + 1;

% Workspace anpassen, Ausreißer ausschließen (remove outlier)
%for each element of the 4th dimension of workspace matrix
% for i = 1 : grid_n + 1
%     for j = 1 : grid_n +1
%         for k = 1 : grid_n +1
%         if workspace_adapt(i, j, k) == 1 %go through all of the workspace adapt to find if element is included in workspace (==1)
%             %check 8 neighbors
%             neighbor = zeros(8,2); %preallocating for speed
%             neighbor(1:8,1:3) = [i+[-1;0;3;-1;1;-1;0;1] j+[-1;-1;-1;0;0;1;1;1] k+[-1;-1;-1;0;0;1;1;1]]; %define its neighbors
%             neighbor = neighbor(all(neighbor,2) & neighbor(:,1) <= grid_size & neighbor(:,2) <= grid_size,:); %get only neighbors within the array
%             for i = 1 : size(neighbor, 1)
%                 neighbor(i, 3) = workspace_adapt(neighbor(i, 1), neighbor(i, 2)); %3. Spalte Workspace Zugehörigkeit
%             end
%             %if all neighbors 0 in third column, remove it
%             if any(neighbor(:, 3)) == 0
%                 workspace_adapt(i, j) = 0; %remove element from workspace, bc not reachable
%             end
%         end
%         end
%     end
% end

workspace_further_adapt_struct = bwconncomp(workspace_adapt); %Find and count connected components in binary image, default connectivity is 8.
workspace_further_adapt_cell = struct2cell(workspace_further_adapt_struct);

[nrows,ncols] = cellfun(@size, workspace_further_adapt_cell{4,1}); %find the number of row & column of the connected components in binary image (Pixel Index List)
max_object = max(nrows); %max nrow =3113 
max_object = find(nrows == max_object); %find the nrows exactly as 3113, which is one. 

if isempty(max_object) %Determine whether array is empty, returns logical 1 (true) if A is empty, and logical 0 (false) otherwise
    workspace_further_adapt = zeros(grid_n + 1, grid_n + 1); %assign all to zeros
    workspace_adapt_pointwise = [-500 500]; %for total of 1000 points
else
    workspace_further_adapt_temp = workspace_further_adapt_cell{4,1}{1,max_object}; %D= 3113x1 double, all the position (Position 69, 70, 132, 133...) of connected components has been listed in 3113x1 matrix. 
    %  = cell2mat(workspace_further_adapt_cell{4,1}{1,max_object});
    workspace_further_adapt = zeros(grid_n + 1, grid_n + 1,grid_n + 1);  %preallocation for speed
    workspace_further_adapt(workspace_further_adapt_temp) = 1; %go thorugh the matrix with position of connected components, then change them to one
end

%% Fläche berechnen, speichern
%transform adapted workspace array to pointwise indices
sz = [grid_size grid_size grid_size]; %define the size of the matrix 
index_convexhull_point = find(workspace_further_adapt == 1); 

%Go through the index of convexhull point, then find the coordinate in x,y and z 
for j = 1 : length(index_convexhull_point)
    [id_x, id_y, id_z] = ind2sub(sz, index_convexhull_point(j)); %Convert linear indices to subscripts with dimension of grid_size(67)
    workspace_adapt_pointwise(j, 1) = workspace(id_x, 1); %select x-coordinate from workspace 
    workspace_adapt_pointwise(j, 2) = workspace(id_y, 2); %select y-coordinate from workspace 
    workspace_adapt_pointwise(j, 3) = workspace(id_z, 3); %select z-coordinate from workspace 
end

%get convex hull of current MU
% [k, convexhull_area] = convhull(workspace_adapt_pointwise);


frac_area_of_1 = sum(workspace_further_adapt(:)); %define the 'workspace_further_adapt' into a spaltenvektor (Dimension: 4489x1) %./numel(workspace_adapt_pointwise);

%% Schwerpunkt berechnen, speichern 
% Method 1, using mean
% 
% sp_row = 1 : size(workspace_further_adapt, 1);  %Schwerpunkt row
% sp_column = 1 : size(workspace_further_adapt, 2); %Schwerpunkt column 
% [SP_column, SP_row] =meshgrid(sp_column, sp_row); %generate SP_column (67x67) in x-coordinate & SP_row (67x67) in y-coordinate
% meanA = mean(workspace_further_adapt(:)); 
% centerOfMasscolumn = mean(workspace_further_adapt(:) .* SP_column(:)) / meanA;
% centerOfMassrow = mean(workspace_further_adapt(:) .* SP_row(:)) / meanA;
% %% Speichern in analysis array
% %Platform Konfig
% analysis(counter_analysis, 1) = b_name;
% %Minimale Seilkraft
% analysis(counter_analysis, 2) = f_min;
% %Maximale Seilkraft
% analysis(counter_analysis, 3) = f_max;
% %Rotation der Plattform
% analysis(counter_analysis, 4) = rot_name;
% % %Fläche des Arbeitsraumes
% % analysis(counter_analysis, 5) = convexhull_area;
% %Fläche des Arbeitsraumes Anteilsmäßig aus Anteil 1en
% analysis(counter_analysis, 5) = frac_area_of_1;
% %Schwerpunkt des Arbeitsraumes Row 
% analysis(counter_analysis, 6) = centerOfMassrow;
% %Schwerpunkt des Arbeitsraumes Column 
% analysis(counter_analysis, 7) = centerOfMasscolumn;


%% Plots
%plot Konvexe Hülle und speichern
figure(counter_analysis)
plot3(workspace_adapt_pointwise(:,1), workspace_adapt_pointwise(:,2),workspace_adapt_pointwise(:,3), '.g'); %Plot x- and y-coordinate
grid on
view(3)
% [caz,cel] = view
v = [-5 -5 6];
[caz,cel] = view(v);
% hold on
% plot(workspace_adapt_pointwise(k, 1), workspace_adapt_pointwise(k, 2), 'r');

%plot Rahmen
% hold on
% plot3(a(1, :), a(2, :),a(3,:), 'xk'); %plot the frame with marker 'x' and black colour 'k'
% hold on
a_adapt = a;
a_adapt(1,5) = a(1,1); %extend to fifth column (so that the rectangle close up)
a_adapt(2,5) = a(2,1); %extend to fifth column (so that the rectangle close up)
a_adapt(3,5) = a(3,1);
% plot(a_adapt(1, :), a_adapt(2, :), 'k--'); %plot the rechtangle (with black colour 'k' & marker '--')
% hold on
% plot(b(1, :), b(2, :), 'xk');

%plot Endeffektor
hold on
b_figure = R * b; %Rotation * base(end-effector)
b_figure(1,5) = b_figure(1,1); %extend to fifth column (so that the rectangle close up)
b_figure(2,5) = b_figure(2,1); %extend to fifth column (so that the rectangle close up)
b_figure(3,5) = b_figure(3,1);
plot3(b_figure(1, :), b_figure(2, :),b_figure(3, :), 'x--k');

%plot Seile
for i = 1 : 4
    hold on
    plot3([a_adapt(1, i) b_figure(1, i)], [a_adapt(2, i) b_figure(2, i)], [a_adapt(3, i) b_figure(3, i)],'--r');
end
% axis([-400 400 -400 400]) %axis in x_min, x_max and y_min, y_max 
% xticks([-350 0 350]) %label of x-coordinate
% yticks([-350 0 350]) %label of y-coordinate
axis square  %Use axis lines with equal lengths. Adjust the increments between data units accordingly.
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate

% filename = "1R2T_workspace_convhull_%d_%d_%d_%d";
% path = sprintf(filename, b_name, rot_name, w_p, w_p_t);
% saveas(figure(counter_analysis), path, 'png');
% close(figure(counter_analysis))


%% Speichern
% filename_excel = "Workspace_analysis.xlsx";
% path_excel = sprintf(filename_excel);

% %Sheet : analysis
% excel_save = "Rahmen1_%d_%d_%d_%d_%d_%d_%d_%d.xlsx";
% excel_name = sprintf(excel_save, b_name, f_min, f_max, rot_name, w_p, f_g, grid_n, grid_deg);
% writematrix(analysis, excel_name);


% SAVE WORKSPACE
% filename_save = "1R2T_Rahmen1_%d_%d_%d_%d.mat";
% path_save = sprintf(filename_save, b_name, rot_name, w_p, w_p_t);
% save(path_save);
end 

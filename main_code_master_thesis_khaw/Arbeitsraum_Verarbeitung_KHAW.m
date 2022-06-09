%% Function Arbeitsraum_Verarbeitung
function [analysis, workspace_logical, workspace_adapt_pointwise] = Arbeitsraum_Verarbeitung_KHAW(a, b, grid_n, b_name,  w_p, w_p_t, f_g, counter_analysis, rot_name, analysis, coordinate, workspace_logical, R, grid_deg)
%% AUSWERTUNG Arbeitsraum
workspace_adapt = workspace_logical; %workspace_logical has the final logic of all wrench-frasible working space in 3D
global f_min f_max noC %import global variable 

%% calculate relative neighbours (27-1) point for 3D.
ind = (1:27)'; 
sz = [3 3 3];
[x,y,z]= ind2sub(sz,ind);

relative_neighbor = [x y z]-2 ; %minus two because the first coordinate start with (-1,-1,-1), see the cube dimension in draw.io
relative_neighbor(14,:)=[]; %the absolute coordinate (in position 5, layer 2) must be removed 

%% Workspace anpassen, Ausreißer ausschließen (remove outlier) for each
%element of the 4th dimension of workspace matrix

for i = 1 : length(coordinate.x) %for coordinate x
    for j = 1 : length(coordinate.y)%for coordinate y
        for k = 1 : length(coordinate.z) %for coordinate z
            if workspace_adapt(i, j, k) == 1 %go through all of the workspace adapt to find if element is included in workspace (==1 TRUE)
            
            %DO NOT DELETE these comments 
            %            Alternative methode for introducing z in 3Dimension
            %               neighbor = zeros(27,3); %preallocating for speed
                            %for page=1:3
            %                neighbor((page-1)*9+1:page*9,:) =[i+[-1;0;1;-1;0;1;-1;0;1] j+[-1;-1;-1;0;0;0;1;1;1] k+page*ones(9,1)-2]; %define its neighbors
            %               end
             
            neighbor = relative_neighbor + [i j k]; %surrounding 26 points + absolute coordinate in 3D space
            
            %To check if the coordinate of all 26 absolute neighbours are on the
            %border (26x26x26). If any abs. neighbour are on the border, then the whole row will be logical false and ignored.  
            compare = (all(neighbor~=0,2)) & (neighbor(:,1) <= length(coordinate.x))...
                  & (neighbor(:,2) <= length(coordinate.y)) & (neighbor(:,3) <= length(coordinate.z)); 
            neighbor = neighbor(compare,:); %get only neighbors within the array 
            workspace_neighbor = zeros(1,26);
            
                for n = 1 : size(neighbor, 1)
                    workspace_neighbor(n) = workspace_adapt(neighbor(n, 1), neighbor(n, 2), neighbor(n,3)); %get the coordinate of the neighbour within the workspace %3. Spalte Workspace Zugehörigkeit
                end
            
            %if all neighbors 0 (all 26 points),then the position in workspace_adapt is Zero. 
                if all(workspace_neighbor == 0) 
                     workspace_adapt(i, j, k) = 0; %remove element from workspace, bc not reachable
                end
            end
        end
    end
end


%To choose the bigger area if there are two point cloud area exist at the
%same time
workspace_further_adapt_struct = bwconncomp(workspace_adapt); %Find and count connected components in binary image, default connectivity is 8.
workspace_further_adapt_cell = struct2cell(workspace_further_adapt_struct);
% pixel = workspace_further_adapt_struct.PixelIdxList{1, 1};

[nrows,ncols] = cellfun(@size, workspace_further_adapt_cell{4,1}); %find the number of row & column of the connected components in binary image (Pixel Index List)
max_object = max(nrows); %max nrow =3113 
max_object = find(nrows == max_object); %find the nrows exactly as 3113, which is one. 

if isempty(max_object) %Determine whether array is empty, returns logical 1 (true) if A is empty, and logical 0 (false) otherwise
    workspace_further_adapt = zeros(grid_n, grid_n); %assign all to zeros
    workspace_adapt_pointwise = [0 0 0]; %random point, to plot outside of the area 
else
    workspace_further_adapt_temp = workspace_further_adapt_cell{4,1}{1,max_object}; %D= 3113x1 double, all the position (Position 69, 70, 132, 133...) of connected components has been listed in 3113x1 matrix. 
    %  = cell2mat(workspace_further_adapt_cell{4,1}{1,max_object});
    workspace_further_adapt = zeros(grid_n, grid_n, grid_n);  %preallocation for speed
    workspace_further_adapt(workspace_further_adapt_temp) = 1; %go thorugh the matrix with position of connected components, then change them to one
end


%% Fläche berechnen, speichern
%transform adapted workspace array to pointwise indices
sz = [length(coordinate.x), length(coordinate.y), length(coordinate.z)]; %define the size of matrix for command ind2sub 
index_convexhull_point = find(workspace_further_adapt == 1); 

%Go through the index of convexhull point, then find the coordinate in x,y and z 
for j = 1 : length(index_convexhull_point)
    [id_x, id_y, id_z] = ind2sub(sz, index_convexhull_point(j)); %Convert linear indices to subscripts with dimension of grid_size(67)
    workspace_adapt_pointwise(j, 1) = coordinate.x(id_x); %select x-coordinate from workspace 
    workspace_adapt_pointwise(j, 2) = coordinate.y(id_y); %select y-coordinate from workspace 
    workspace_adapt_pointwise(j, 3) = coordinate.z(id_z); %select z-coordinate from workspace 
end

%% get convex hull of current working space in extra figure
[k, convexhull_volume] = convhull(workspace_adapt_pointwise,'Simplify',true);
figure
trisurf(k,workspace_adapt_pointwise(:,1),workspace_adapt_pointwise(:,2),workspace_adapt_pointwise(:,3),'FaceColor','green')
axis equal

%add Title workaround methode
formatSpec = "The current workspace is: %e %s";
A1 = convexhull_volume*1e-9; %1e-9 for changing from mm3 to m3 
A2 = 'm3';
str = sprintf(formatSpec,A1,A2)
title(str)

xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate

    %To plot the Data from WireX
    % hold on 
    % patch(DAT(1:3,:),DAT(4:6,:),DAT(7:9,:),1,'LineWidth',1)

%% 
frac_area_of_1 = sum(workspace_further_adapt(:)); %define the 'workspace_further_adapt' into a spaltenvektor (Dimension: 4489x1) %./numel(workspace_adapt_pointwise);

%% Schwerpunkt berechnen, speichern 
% Method 1, using mean
% 
sp_row = 1 : size(workspace_further_adapt, 1);  %Schwerpunkt row
sp_column = 1 : size(workspace_further_adapt, 2); %Schwerpunkt column 
sp_page = 1: size(workspace_further_adapt, 3); %Schwerpunkt page
[SP_row, SP_column, SP_page] =meshgrid(sp_column, sp_row, sp_page); %generate SP_column (67x67) in x-coordinate & SP_row (67x67) in y-coordinate
meanA = mean(workspace_further_adapt(:)); 
centerOfMasscolumn = mean(workspace_further_adapt(:) .* SP_column(:)) / meanA;
centerOfMassrow = mean(workspace_further_adapt(:) .* SP_row(:)) / meanA;
centerOfMasspage = mean(workspace_further_adapt(:) .* SP_page(:)) / meanA;
%% Speichern in analysis array
%Platform Konfig
analysis(counter_analysis, 1) = b_name;
%Minimale Seilkraft
analysis(counter_analysis, 2) = f_min;
%Maximale Seilkraft
analysis(counter_analysis, 3) = f_max;
%Rotation der Plattform
analysis(counter_analysis, 4) = rot_name;
%Fläche des Arbeitsraumes Anteilsmäßig aus Anteil 1en
analysis(counter_analysis, 5) = frac_area_of_1;
%Schwerpunkt des Arbeitsraumes Row  (x)
analysis(counter_analysis, 6) = centerOfMassrow;
%Schwerpunkt des Arbeitsraumes Column (y)
analysis(counter_analysis, 7) = centerOfMasscolumn;
%Schwerpunkt des Arbeitsraumes Page (z)
analysis(counter_analysis, 8) = centerOfMasspage;
%Fläche des Arbeitsraumes
analysis(counter_analysis, 9) = convexhull_volume;


%% Plots Working space
%plot Konvexe Hülle und speichern
% figure(counter_analysis)
figure
plot3(workspace_adapt_pointwise(:,1), workspace_adapt_pointwise(:,2),workspace_adapt_pointwise(:,3), '.g','LineWidth',5); %Plot x- and y-coordinate
grid on
grid minor

%View on three different plane
% view(0,90)  % XY
% pause
% view(0,0)   % XZ
% % pause
% view(90,0)  % YZ

%plot Rahmen
a_adapt = a;
% a_adapt(1:3, noC+1) = a(1:3,1); %extend to next column (so that the rectangle close up)


%Plot Rahmen (KHAW) 
box =  [-350  450  0 %define the coordinate of the box
       350    450   0
       350   -450   0
      -350   -450   0
       -350   450   250
       350    450   250
       350   -450   250
      -350   -450   250];
  
idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]'; %the ascending index of the box that will be plotted one after another 

xc = box(:,1);
yc = box(:,2);
zc = box(:,3);

%Plot one or more filled polygonal regions with facealpha = semitransparent polygons 
patch(xc(idx), yc(idx), zc(idx), 'r', 'facealpha', 0.1); 
view(3); %3D view

%plot Endeffektor
hold on
b_figure = R * b; %Rotation * base(end-effector)
b_figure (:,noC+1) = b_figure(1:3,1); %extend to next column (so the shape can close up)
plot3(b_figure(1, :), b_figure(2, :),b_figure(3, :), 'x-k','LineWidth',2);

%plot Seile
str = ["w1" "w2" "w3" "w4" "w5" "w6" "w7" "w8"]; 
for i = 1 : noC
    hold on
    plot3([a_adapt(1, i) b_figure(1, i)], [a_adapt(2, i) b_figure(2, i)], [a_adapt(3, i) b_figure(3, i)],'--r');
    text(a_adapt(1, i), a_adapt(2,i), a_adapt(3,i), str(i)); %add the label on each cable 
end

% axis([-400 400 -400 400]) %axis in x_min, x_max and y_min, y_max 
% xticks([-350 0 350]) %label of x-coordinate
% yticks([-350 0 350]) %label of y-coordinate
% axis square  %Use axis lines with equal lengths. Adjust the increments between data units accordingly.
title('Wrench-feasible working space in cable-driven Robotics')
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


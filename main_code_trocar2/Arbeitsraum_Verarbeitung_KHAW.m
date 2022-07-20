%% Function Arbeitsraum_Verarbeitung
function [analysis, workspace_logical, workspace_adapt_pointwise] = Arbeitsraum_Verarbeitung_KHAW(a, b, grid_n, b_name,  w_p, w_p_t, f_g, counter_analysis ,rot_name, analysis, coordinate, workspace_logical, R, grid_deg)
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
            % Alternative methode for introducing z in 3Dimension
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


%To choose the bigger area if there are two point cloud area exist at the same time
workspace_further_adapt_struct = bwconncomp(workspace_adapt); %Find and count connected components in binary image, default connectivity is 8.
workspace_further_adapt_cell = struct2cell(workspace_further_adapt_struct);
% pixel = workspace_further_adapt_struct.PixelIdxList{1, 1};

[nrows,~] = cellfun(@size, workspace_further_adapt_cell{4,1}); %find the number of row & column of the connected components in binary image (Pixel Index List)
max_object = max(nrows); %max nrow =3113 
max_object = find(nrows == max_object); %find the nrows exactly as 3113, which is one. 

if isempty(max_object) %Determine whether array is empty, returns logical 1 (true) if A is empty, and logical 0 (false) otherwise
disp('Fehler')
    workspace_further_adapt = zeros(length(coordinate.x), length(coordinate.y), length(coordinate.z)); %assign all to zeros
    workspace_adapt_pointwise = [0 0 0]; %random point, to plot in the middle 
else
    workspace_further_adapt_temp = workspace_further_adapt_cell{4,1}{1,max_object}; %D= 3113x1 double, all the position (Position 69, 70, 132, 133...) of connected components has been listed in 3113x1 matrix. 
    %  = cell2mat(workspace_further_adapt_cell{4,1}{1,max_object});
    workspace_further_adapt = zeros(length(coordinate.x), length(coordinate.y), length(coordinate.z));  %preallocation for speed
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
%{
[k, convexhull_volume] = convhull(workspace_adapt_pointwise,'Simplify',true);
figure
trisurf(k,workspace_adapt_pointwise(:,1),workspace_adapt_pointwise(:,2),workspace_adapt_pointwise(:,3),'FaceColor','b','Edgecolor','b')
% axis equal
%Define the projection onto the wall 
y_plane = max(workspace_adapt_pointwise(:,2))+150; %from maximum of y_plane coordinate +200 mm (show on right)
x_plane = max(workspace_adapt_pointwise(:,1))+150; %from maximum of x_plane coordinate +200 mm (show on left)
z_plane = max(workspace_adapt_pointwise(:,2))-400; %from maximum of z_plane coordinate -150 mm (show on bottom) 
hold on
grid on 
grid minor
trisurf(k,workspace_adapt_pointwise(:,1), y_plane*ones(size(workspace_adapt_pointwise(:,2))), workspace_adapt_pointwise(:,3),'FaceColor','r','Edgecolor','r'); % project in x-z axis at y=100
trisurf(k,x_plane*ones(size(workspace_adapt_pointwise(:,1))), workspace_adapt_pointwise(:,2), workspace_adapt_pointwise(:,3),'FaceColor','c','Edgecolor','c'); % project in y-z axis at x=2
trisurf(k,workspace_adapt_pointwise(:,1), workspace_adapt_pointwise(:,2), z_plane*ones(size(workspace_adapt_pointwise(:,3))),'FaceColor','g','Edgecolor','g'); % project in x-y axis at z=-2

 %Calculate the percentage of Workspace within ROI
 [percentage] = Workspace_in_ROI(workspace_adapt_pointwise); 

%add Title workaround methode
formatSpec = "The current workspace is: %e %s";
format long g
current_ws = convexhull_volume*1e-9; %1e-9 for changing from mm3 to m3
unit = 'm3';
str = sprintf(formatSpec,current_ws,unit);
title(str)
str2 = [num2str(percentage) '% of Workspace was in ROI'];
subtitle(str2)
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate
%}

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

%%Calculate the Volume of the frame
length_frame = max(a(1,:))-min(a(1,:)); %x-axis, use long insted of length cause length has been used before
width_frame = max(a(2,:))-min(a(2,:)); %y-axis
%height_frame = max(a(3,:))-min(a(3,:)); %z-axis 
height_rod = max(b(3,:))-min(b(3,:)); %length of the rod in total
Volume_frame = length_frame*width_frame*height_rod*1e-9; 


%% Speichern in analysis array
%Platform Konfig
analysis(counter_analysis, 1) = b_name;
%Minimale Seilkraft
% analysis(counter_analysis, 2) = f_min;
% %Maximale Seilkraft
% analysis(counter_analysis, 3) = f_max;
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
% analysis(counter_analysis, 9) = convexhull_volume;

%Volume of the frame 
analysis(counter_analysis,10) = Volume_frame;

%current Workspace
% analysis(counter_analysis,11) = current_ws;

%Ratio of Volume to total Workspace
% analysis(counter_analysis,12) = Volume_frame./current_ws;



%% Plots Working space
%plot Konvexe Hülle und speichern
figure()
plot3(workspace_adapt_pointwise(:,1), workspace_adapt_pointwise(:,2),workspace_adapt_pointwise(:,3), '.g','LineWidth',8); %Plot x- and y & z-coordinate
hold on 
grid on
grid minor
daspect([1,1,1])

%%Plot Trocar point at Origin
plot3(0,0,0,'bo','LineWidth',5)
hold on 

%%Plot Region of Interest (ROI)
r = 150; %radius in mm 
[X,Y,Z] = cylinder(r);
% X = X+90;
% Y = Y+100;
h = 200; %height in mm
Z = (Z*h)-100; %minus 100 so that its from -100 to 100 in Z-axis
surf(X,Y,Z,'FaceColor','r','FaceAlpha','0.3')
hold on 

%plot Rahmen
a_adapt = a;
% a_adapt(1:3, noC+1) = a(1:3,1); %extend to next column (so that the rectangle close up)

%{
%Plot Rahmen (KHAW) 
box =  [-0.25  0.1875  0.125 %define the coordinate of the box
       0.25    0.1875   0.125
       0.25   -0.1875   0.125
      -0.25   -0.1875   0.125
       -0.25   0.1875   0.0625
       0.25    0.1875   0.0625
       0.25   -0.1875   0.0625
      -0.25   -0.1875   0.0625];
box = box.*1000;
  
idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]'; %the ascending index of the box that will be plotted one after another 

xc = box(:,1);
yc = box(:,2);
zc = box(:,3);


%Plot one or more filled polygonal regions with facealpha = semitransparent polygons 
patch(xc(idx), yc(idx), zc(idx), 'r', 'facealpha', 0.1); 
view(3); %3D view
%}

%plot Endeffektor
hold on
b_figure = R * b; %Rotation * base(end-effector)
b_5 = b_figure(:,1); %so that it close up the endeffector upper side
b_10 = b_figure(:,5); %so that it close up the endeffector lower side
b_figure_new = [b_figure(:,1:4) b_5 b_figure(:,5:8) b_10];
plot3(b_figure_new(1, :), b_figure_new(2, :),b_figure_new(3, :), 'x-k','LineWidth',2);
%plot straight line for endeffector 
% w2 = b_figure_new (:,2);
% w7 = b_figure_new (:,7);
% w3 = b_figure_new (:,3);
% w8 = b_figure_new (:,8);
% w4 = b_figure_new (:,4);
% w9 = b_figure_new (:,9);
% line = [w2 w7 w3 w8 w4 w9];
%  plot3(line(1,:),line(2,:),line(3,:), 'x-k','LineWidth',2)



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
length_frame = max(a(1,:))-min(a(1,:)); %x-axis, use long insted of length cause length has been used before
width_frame = max(a(2,:))-min(a(2,:)); %y-axis
height_frame = max(a(3,:))-min(a(3,:)); %z-axis 
height_rod = max(b(3,:))-min(b(3,:));

title('Wrench-feasible working space in cable-driven input device')
txt = ['L= ' int2str(length_frame) ' x W= ' int2str(width_frame) ' x H= ' int2str(height_frame) ' Rod= ' int2str(height_rod) ' [mm] Rotation = ' int2str(rot_name) '° ' 'wp = ' int2str(w_p)];
subtitle(txt)
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate


% %%Save 3d figure to file 
% folder = 'D:\Masterarbeit\11_MATLAB_GIT\Figure';
%   baseFileName = "Feasible_Workspace_%d_%d";
%   path = sprintf(baseFileName, b_name, height_rod); %current working directory 
%   saveas(figure(counter_analysis), path, 'png'); %save as (filename,variable,format)
%   close(figure(counter_analysis))

%   %%Save projection figure
%   baseFileName = "Feasible_Workspace_projection_%d_%d";
%   path = sprintf(baseFileName, b_name, height_rod); %current working directory 
%   saveas(figure(counter_analysis_proj), path, 'png'); %save as (filename,variable,format)
%   close(figure(counter_analysis_proj))


 % fullFileName = fullfile(folder,baseFileName);
% saveas(figure(counter_analysis),[pwd '/Figure/myFig.fig']);

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


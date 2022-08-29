function [r] = ws_plot_khaw(workspace_trans_mat_total,a,b,noC)
%plot workspace (rotation x-y axis and translation in z-axis) 

figure 

%% Plot Trocar point at Origin
plot3(0,0,0,'bo','LineWidth',5)
hold on 

%plot Rod 
Rod_top = [0 0 300];
Rod_bottom = [0 0 -300];
Rod = [Rod_top; Rod_bottom]';
plot3(Rod(1,:), Rod(2,:), Rod(3,:),'b','LineWidth',2)
hold on 

%% Plot Region of Interest (ROI)
r = 150; %radius in mm 
[X,Y,Z] = cylinder(r);
X = X-0;
Y = Y-0;
h = 200; %height in mm
Z = (Z*h)-400; %minus 100 so that its from -100 to 100 in Z-axis
surf(X,Y,Z,'FaceColor','w','FaceAlpha','0.3')

%% Plot Seile 
% a_adapt = a;
a_figure = [a(:,1:4) a(:,8) a(:,5:8) a(:,5)]; %focus column 5 and column 10 
b_figure = [b(:,1:4) b(:,8) b(:,5:8) b(:,5)];

str = ["w1" "w2" "w3" "w4" "w5" "w6" "w7" "w8"]; 
for i = 1 : noC
    plot3([a_figure(1, i) b_figure(1, i)], [a_figure(2, i) b_figure(2, i)], [a_figure(3, i) b_figure(3, i)],'--r');
    text(a_figure(1, i), a_figure(2,i), a_figure(3,i), str(i)); %add the label on each cable 
end


%% plot endeffector 
b_figure_end = [b(:,1:4) b(:,1) b(:,5:8) b(:,5)];
plot3(b_figure_end(1, 1:5), b_figure_end(2, 1:5),b_figure_end(3, 1:5), 'x-k','LineWidth',2); %plot the frame of end-effector only top 
plot3(b_figure_end(1, 6:10), b_figure_end(2, 6:10),b_figure_end(3, 6:10), 'x-k','LineWidth',2); %plot the frame of end-effector only bottom

%% Plot Frame
%Plot Rahmen (KHAW) 
box = a';
idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]'; %the ascending index of the box that will be plotted one after another 
xc = box(:,1);
yc = box(:,2);
zc = box(:,3);

%Plot one or more filled polygonal regions with facealpha = semitransparent polygons 
patch(xc(idx), yc(idx), zc(idx), 'w', 'facealpha', 0.1); 

%% Plot WORKSPACE_TRANSLATION
plot3(workspace_trans_mat_total(:,1),workspace_trans_mat_total(:,2),workspace_trans_mat_total(:,3),'.g')
daspect([1,1,1]) %For equal data unit lengths in all directions

%% Title of plot
length_frame = max(a(1,:))-min(a(1,:)); %x-axis
width_frame = max(a(2,:))-min(a(2,:)); %y-axis
height_frame = max(a(3,:))-min(a(3,:)); %z-axis 
height_rod = max(b(3,:))-min(b(3,:));

title('Rotational workspace in Cable-Driven Haptic Device')
txt = ['L= ' int2str(length_frame) ' x W= ' int2str(width_frame) ' x H= ' int2str(height_frame) ' Rod= ' int2str(height_rod) ' [mm]'];
subtitle(txt)
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate
end
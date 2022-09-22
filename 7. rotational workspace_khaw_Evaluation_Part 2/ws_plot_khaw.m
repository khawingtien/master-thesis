function [vol_results] = ws_plot_khaw(results,a,b,noC) %wp,wpt temporary ommitted
%plot workspace (rotation x-y axis and translation in z-axis) 

% figure 
%% Plot Trocar point at Origin
plot3(0,0,0,'bo','LineWidth',5)
hold on 

%plot Rod 
Rod_top = [0 0 b(3,1)];
Rod_bottom = [0 0 b(3,5)];
Rod = [Rod_top; Rod_bottom]';
plot3(Rod(1,:), Rod(2,:), Rod(3,:),'b','LineWidth',2)
hold on 

%% Plot Region of Interest (ROI)
% r = 150; %radius in mm 
% [X,Y,Z] = cylinder(r);
% X = X-0;
% Y = Y-0;
% h = 200; %height in mm
% Z = (Z*h)-400; %minus 100 so that its from -100 to 100 in Z-axis
% surf(X,Y,Z,'FaceColor','r','FaceAlpha','0.3')

%% Plot Seile 
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
plot3(results(:,1),results(:,2),results(:,3),'.r')
daspect([1,1,1]) %For equal data unit lengths in all directions
grid minor 

%% Plot Boundary in 3D
[k,vol_results] = boundary(results, 1); %in mm3
trisurf(k,results(:,1),results(:,2),results(:,3),'FaceColor','yellow','FaceAlpha',0.1)
vol_results_m3 = vol_results*1e-9; %in m3

%% Title of plot
length_frame = max(a(1,:))-min(a(1,:)); %x-axis
width_frame = max(a(2,:))-min(a(2,:)); %y-axis
height_frame = max(a(3,:))-min(a(3,:)); %z-axis 
height_rod = max(b(3,:))-min(b(3,:));

title('Rotational Workspace in Cable-Driven Haptic Device')
% txt_1 = ['L= ' int2str(length_frame) ' W= ' int2str(width_frame) ' H= ' int2str(height_frame) ' Rod= ' int2str(height_rod) ' [mm] wp= ' int2str(w_p) ' wpt = ' int2str(w_p_t)  ' [N]'];
txt_1 = ['L = ' int2str(length_frame) ' W = ' int2str(width_frame) ' H = ' int2str(height_frame) ' Rod = ' int2str(height_rod) ' [mm]'];
txt_2 = ['Volume = ' num2str(vol_results_m3) ' [m^3]'];
subtitle({txt_1,txt_2})

xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate


% %% Legend of plot with 4 Spec 
% dim = [.8 .6 .6 .3];
% output1 = ['Ivv = ',num2str(round(I_vv_value,2))];
% output2 = ['Volume ws = ',num2str(round(Volume_ws_value,3)),' [m^3]'];
% output3 = ['Volume frame = ',num2str(round(Volume_frame_value,3)),' [m^3]'];
% output4 = ['Percentage = ',num2str(round(Percentage_value,2)),' [%]'];
% mytext = {output1,output2,output3,output4};
% annotation('textbox',dim,'String',mytext,'FitBoxToText','on')
% 

end
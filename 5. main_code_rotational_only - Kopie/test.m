figure

coordinate_z = -150:30:150;

%%Plot Trocar point at Origin
plot3(0,0,0,'bo','LineWidth',5)
hold on 

for index = 1:length(coordinate_z)
coord = coordinate_z(index);
R = axang2rotm([1 0 0 deg2rad(10)]);

b_figure = (R*b);
b_figure = b_figure - [0 0 coord]';

b_figure_new = [b_figure(:,1:4), b_figure(:,1), b_figure(:,5:8), b_figure(:,5)];
plot3(b_figure_new(1, 1:5), b_figure_new(2, 1:5),b_figure_new(3, 1:5), 'x-k','LineWidth',2); %plot the frame of end-effector only top 
hold on 
plot3(b_figure_new(1, 6:10), b_figure_new(2, 6:10),b_figure_new(3, 6:10), 'x-k','LineWidth',2); %plot the frame of end-effector only bottom
hold on 



%%plot the Rod btw. 2 Endeffector 
b_middle_top = R* [[0;0;b(3,1)], [0; 0; b(3,5)]]- [0 0 coord]'; %rotation of the center line through trocar point
plot3(b_middle_top(1,:),b_middle_top(2,:),b_middle_top(3,:) ,'b','LineWidth',2) %plot middle line through trocar point

end


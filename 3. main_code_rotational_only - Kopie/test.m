% coordinate.x = 0;
% coordinate.y = 0;
% coordinate.z = -300;
clear 
clc
Origin = [0;0;0];
position = [0;0;-300];
ws_position = [];
f_directions = ["x","y"]; %define the f_x and f_y wrench direction.

figure
grid on 

plot3(position(1),position(2),position(3),'bo') % end of end-effector 
hold on 
plot3(0,0,0,'ro') %origin 
hold on 
% Coor = [coordinate.x; coordinate.y;coordinate.z];

for f_xy=1:2 
f_direction = f_directions(f_xy);
    for i=1:30
    
switch f_direction
    case 'x' 
    rotation = [1 0 0 -deg2rad(i)]; %rotation at x-axis
    R = axang2rotm(rotation);
    b_middle = R* [[0;0;300], [0; 0; -300]]; %rotation of the center line through trocar point
    plot3(b_middle(1,:),b_middle(2,:),b_middle(3,:) ,'b','LineWidth',2) %plot middle line through trocar point
    plot3(b_middle(1,:),-b_middle(2,:),b_middle(3,:) ,'g','LineWidth',2) %only y-value *-1 (for negative value)
    case 'y'
    rotation = [0 1 0 deg2rad(i)]; %rotation at y-axis  
    R = axang2rotm(rotation);
    b_middle = R* [[0;0;300], [0; 0; -300]]; %rotation of the center line through trocar point
    plot3(b_middle(1,:),b_middle(2,:),b_middle(3,:) ,'b','LineWidth',2) %plot middle line through trocar point
    plot3(-b_middle(1,:),b_middle(2,:),b_middle(3,:) ,'r','LineWidth',2) %only x-value *-1
end

ws_position = [ws_position b_middle(:,2)]; 
    end
end

daspect([1,1,1]) %For equal data unit lengths in all directions
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate
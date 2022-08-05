function [ws_bowl_mat] = ws_position_bowl()

position = [0;0;-300];
ws_orientation = [];
ws_bowl_mat = [];

figure
plot3(position(1),position(2),position(3),'bo') % end of end-effector 
hold on 
plot3(0,0,0,'ro') %origin 
hold on

%Rotation of all orientation around x-axis
    for i=1:31 %orientation 0 to 30°
        rotation = [1 0 0 deg2rad(i-1)]; %rotation at x-axis
        R = axang2rotm(rotation);
        rod_middle = R* [[0;0;300], [0; 0; -300]]; %rotation of the center line through origin
        plot3(rod_middle(1,:),rod_middle(2,:),rod_middle(3,:) ,'b','LineWidth',2) %plot middle line through trocar point
        ws_orientation = [ws_orientation rod_middle(:,2)]; 
    end

%Rotation of all the orientation (0 to 30°) around z-axis
for angle = 1:10:360
rotation_z = [0 0 1 deg2rad(angle)]; %rotation at z-axis  
    R = axang2rotm(rotation_z); 
    for index = 1:30
    ws_bowl = R* ws_orientation(:,index);
    plot3(ws_bowl(1,:),ws_bowl(2,:),ws_bowl(3,:) ,'ro','LineWidth',1) %plot middle line through trocar point
    ws_bowl_mat = [ws_bowl_mat ws_bowl];
    end
end

grid on 
daspect([1,1,1]) %For equal data unit lengths in all directions
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate
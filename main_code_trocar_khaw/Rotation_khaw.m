%define dimension of the base 
a = [-230 230 230 -230 ; %in mm 
     102 102 -102 -102]; %in mm 

a_close = [a  a([1 2])'];%to close up the rectangle, to connect to the start point

%define the trocar point
trocar = [0;0];

%define rod length
rod_length = 500; %mm 
rod_length_x= zeros(1,501);
rod_length_y = 0:-1:-rod_length;

%calculate the 30° rotation for rod at the trocar (origin)
angle = [60];
A(:, 1) = rod_length_x; %rod length x
A(:, 2) = rod_length_y; %rod length y
% Compute the rotation matrix.
rotationMatrix = [cosd(angle) -sind(angle); sind(angle) cosd(angle)];
% Rotate the coordinates.
rotXY = A *rotationMatrix;
u=rotXY(:, 1); %rotation matrix x-coordinate
v=rotXY(:, 2); %rotation matrix y-coordinate

%plot for visualisation
figure
grid on 
plot(a_close(1,:),a_close(2,:)); %Plot frame 
hold on
plot(trocar,'bo') %plor trocar point 
hold on
plot(rod_length_x, rod_length_y,'r-','LineWidth', 2) %plot Rod in the original position (rotation = 0°)
hold on
axis equal
hold on
plot(u,v,'b-', 'LineWidth', 2) %plot Rod in rotation matrix  
title(['Rotation of ' int2str(angle) ' degree at Origin'])
xlabel('x-coordinate in mm') 
ylabel('y-coordinate in mm') 
grid on;
legend('frame','trocar','Original', 'Rotated')



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

%define endeffector square 
bx = 17; %in mm
by = 380; %in mm
by_zero = 0; 
% bz = 0.38;  %in m
b30 = [bx  bx  -bx    -bx ;
       by_zero  -by  -by   by_zero   ];
%        0.12  0.12   0.12    0.12   -bz    -bz   -bz  -bz];
% b30 = b30.*1000; %in mm
b30_close = [b30  b30([1 2])'];%to close up the rectangle, to connect to the start point


%calculate the 30° rotation for rod at the trocar (origin)
angle = 30;
A(:, 1) = rod_length_x; %rod length x %the rotation point muss remain the same !
A(:, 2) = rod_length_y; %rod length y

%calculate rotation of endeffector right
B_x = ones(380,1)*bx; %the rotation point muss remain the same ! 
B(:,1) = B_x;
B_y = linspace(by_zero, -by, 380); %the length of the endeffector in small steps (top to bottom)
B(:,2) = B_y;

%calculate rotation of endeffector left
C_x = ones(380,1)*-bx; %the rotation point muss remain the same !
C(:,1) = C_x;
C_y = linspace(by_zero, -by, 380); %same as B_y  %the length of the endeffector in small steps (top to bottom)
C(:,2) = C_y;

% Compute the rotation matrix.
rotationMatrix = [cosd(angle) -sind(angle); sind(angle) cosd(angle)];
% Rotate the coordinates.
rotXY = A *rotationMatrix;
u=rotXY(:, 1); %rotation matrix x-coordinate
v=rotXY(:, 2); %rotation matrix y-coordinate

rotXY_effector_right = B *rotationMatrix;
u_effector_right =rotXY_effector_right(:,1);
v_effector_right =rotXY_effector_right(:,2);

rotXY_effector_left = C *rotationMatrix;
u_effector_left =rotXY_effector_left(:,1);
v_effector_left =rotXY_effector_left(:,2);



%plot for visualisation
figure
grid on 
plot(a_close(1,:),a_close(2,:)); %Plot frame 
hold on
plot(trocar,'bo') %plor trocar point 
hold on
plot(rod_length_x, rod_length_y,'r-','LineWidth', 2) %plot Rod in the original position (rotation = 0°)
hold on
plot(b30_close(1,:),b30_close(2,:),'r-','LineWidth', 2) %plot endeffecctor in the original position (rotation = 0°)
axis equal
hold on
plot(u,v,'b-', 'LineWidth', 2) %plot Rod in rotation matrix  
hold on 
plot(u_effector_right,v_effector_right,'b-', 'LineWidth', 2) %plot endeffector_right in rotation matrix
plot(u_effector_left,v_effector_left,'b-', 'LineWidth', 2) %plot endeffector_left in rotation matrix
title(['Rotation of ' int2str(angle) ' degree at Origin'])
xlabel('x-coordinate in mm') 
ylabel('y-coordinate in mm') 
grid on;
legend('frame','trocar','Original', 'Rotated')



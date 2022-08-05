%define dimension of the base 
a = [-230 230 230 -230 ; %in mm 
     102 102 -102 -102]; %in mm 

a_close = [a  a([1 2])'];%to close up the rectangle, to connect to the start point

%define the trocar point
trocar = [0;0];

%define rod length
rod_length = 500; %mm 
rod_length_x = zeros(501,1)*rod_length;
rod_length_y = 0:-1:-rod_length;

%calculate the 180° angle for rod 


%plot for visualisation
figure
grid on 
plot(a_close(1,:),a_close(2,:));
hold on
plot(trocar,'bo')
hold on
plot(rod_length_x, rod_length_y,'r--')

%half circle
xCenter_2 = 0;
yCenter_2 = 0;
theta = 0-1/3 : 0.01 : pi-1/3;
radius = 0.5;
x = radius * cos(theta) + xCenter_2;
y = radius * sin(theta) + yCenter_2;
plot(x, y), hold on;
% axis square;

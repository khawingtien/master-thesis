tic
clear 
clc
counter = 1;

%parameter 
ax = 0.230; %in m
ay = 0.230; %in m 
az = 0.102; %in m

%calculation of frame 
[a] = SetupParameter(ax,ay,az);

%Endeffector matrix
b = endeffektor2();
b =  b{1,1};

%Parameter definition
f_min = 5;
f_max = 36; % fmax berechnet: 2* 183 / 10 = 36, 6 %Motor 
noC = length(a); 
limit.lower = (1/2 * (f_max - f_min)) ; %upper limit for improve closed-form solution (eq. 3.6 Pott book)
limit.upper = (1/2 * sqrt(noC) * (f_max - f_min)); %lower limit for improved closed form (eq. 3.6 Pott book)
f_directions = ["x","y"]; %define the f_x and f_y wrench direction. 

%Pulley definition
pulley_kin = 'no';
rad_pulley = 9.37+0.5;
R_A = 1; %just for input, is not in use 

%wrench definition
w_p_x = 0;
w_p_t = 0;
grid_deg = 8;

% %calculate rotation matrix for wrench 
if w_p_x == 0
    rotation_matrix.wpx = axang2rotm([1 0 0 0]); %Euler Winkel (x,y,z, Winkel), a rotation of 0 radians around the y-axis
    rotation_matrix.wpy = axang2rotm([0 1 0 0]);
else
discrete_rot_angle_w_p = linspace(0, 2*pi*(1-1/grid_deg), grid_deg)'; %(x1, 1/8 from the complete 360°, n) n Punkte zwischen x1 und x2 
    for i = 1 : size(discrete_rot_angle_w_p, 1) %K:calculate each of the rotation
    rotation_matrix.wpx = axang2rotm([1 0 0 (discrete_rot_angle_w_p(i))]); %rotation matrix for f_Y around X-AXIS (IMPORTANT)
    rotation_matrix.wpy = axang2rotm([0 1 0 (discrete_rot_angle_w_p(i))]); %rotation matrix for f_X around Y-AXIS (IMPORTANT)
    end 
end

[position_bowl_mat, R_x_cell, R_z_cell, position_bowl_360] = ws_position_bowl(); %call the function 
ws_logical_bowl = cell(1,length(position_bowl_360)); %preallocating for speed
b_rot_mat = cell(1,360); %preallocating for speed
POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)
figure 


for bowl_arm = 1:length(position_bowl_360)
    for bowl_index = 1:30
        ws_position = position_bowl_360{1,bowl_arm}{1,bowl_index};
        R_x = R_x_cell{1,bowl_index};
       
            R_z = R_z_cell{1,bowl_arm};
            b_rot = R_z* (R_x * b); %endeffector rotation         
%             b_rot = round(((R_z*b)'*R_x)',0); %TEST 
            POI_rot = R_z* (R_x * POI_offset); %the position of the POI after rotation at (0,0,0)

            f_direction = f_directions(1); %ACHTUNG NUR 1 !! TO EDIT 
            [stop] = berechnungSeilkraftverteilung_KHAW(ws_position, a, b, f_min, f_max,noC, b_rot, POI_rot, w_p_x, w_p_t,  rotation_matrix, pulley_kin, rad_pulley, R_A, limit, f_direction);
            
               if stop == 0  %no violation of f_min & f_max (fulfil the requirements)(TRUE)
                  ws_logical_bowl{1,bowl_arm}{1,bowl_index} = 1; %write 1 as TRUE
                  plot3(position_bowl_360{1,bowl_arm}{1,bowl_index}(1),position_bowl_360{1,bowl_arm}{1,bowl_index}(2),position_bowl_360{1,bowl_arm}{1,bowl_index}(3),'ro','LineWidth',1)  %plot rotation     
                  hold on 

               elseif stop == 1 %if violation exist 
                  ws_logical_bowl{1,bowl_arm}{1,bowl_index} = 0; %write 0 as FALSE 
               end   
     end
end

grid on 
daspect([1,1,1]) %For equal data unit lengths in all directions
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate

toc
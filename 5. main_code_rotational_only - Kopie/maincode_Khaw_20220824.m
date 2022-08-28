% close all
clear 
clc
tic %start Stopwatch timer
    figure %open a figure before the for-loop, so that x- and y-plane can be plotted on the same figure 

 %% pulley
pulley_kin = 'no';
ax = 0.230;
ay = 0.230; 
az = 0.102;
[a] = SetupParameter(ax,ay,az);

%% Standardparameter
noC = length(a);
R_A = 1; %just for input, is not in use 
rot_angle_A = 1; %just for input, is not in use  
    
%Define max and min of grid in all direction
grid.x_max = 300; %mm %largest length in x direction
grid.y_max = 300;
grid.z_max = 150;
grid.x_min = -300; %mm %smallest length in x direction
grid.y_min = -300;
grid.z_min = -650;

%Definiere Grid    
grid_n = 20;  %Anzahl der Unterteilungen in X-Richtung
grid_delta = (grid.z_max - grid.z_min) / grid_n;  %step size in x-direction in mm %Gitterabstand von X-Richtung (Y- & Z-Richtung auch in diesem Abstand)

%% Definiere distale Ankerpunkte Plattform [x; y]
b_cell = endeffektor2();
b = b_cell{1, 1};

%% Definiere zu untersuchende Rotationen des Endeffektors um die z-Achse
%  rotation_array_values_z =[0:90];
rotation_array_values_z =[0:5:90];
rotation_array_values_x = [30:-5:0];
rotation_array_x = zeros(length(rotation_array_values_x),4); %preallocationg for speed
rotation_array_z = zeros(length(rotation_array_values_z),4); %preallocationg for speed
rotation_array_xy = zeros(length(rotation_array_values_z),4);
rot_axis=zeros(length(rotation_array_values_z),3);

for i = 1 : length(rotation_array_values_x)
    rotation_array_x(i,:) = [1 1 0 deg2rad(rotation_array_values_x(i))]; %rotation at x-axis 
end

for z = 1: length(rotation_array_values_z)
    rotation_array_z(z,:) = [0 0 1 deg2rad(rotation_array_values_z(z))]; %rotation at z-axis 
end

for i = 1 : length(rotation_array_values_z)
    rot_axis(i,:) = [1 0 0]*axang2rotm(rotation_array_z(i,:)); 
end

% Definiere zu untersuchende Lasten in bestimmte Raumrichtungen definiert durch rotation_w_p
w_p_x = 0; %dieser Wert wird in berechnungSeilkraftverteilung in den wrench Vektor als x-Koordinate eingesetzt, y=0, T=0 (Feedback Kraft in alle Richtung, Translation) 
w_p_t = 0; %Torque (Feedback Kraft in Rotation)%wrench in torque
grid_deg = 8; % rotatorische Auflösung %for 8 only, because 360° is the same as 0°
discrete_rot_angle_w_p = linspace(0, 2*pi*(1-1/grid_deg), grid_deg)'; %(x1, 1/8 from the complete 360°, n) n Punkte zwischen x1 und x2 
rotation_w_array = zeros(size(discrete_rot_angle_w_p, 1), 4);%predefine for speed

%if wrench rotation = zero 
if w_p_x == 0
    rotation_w_array_x = [1 0 0 0]; %Euler Winkel (x,y,z, Winkel), a rotation of 0 radians around the y-axis
    rotation_w_array_y = [0 1 0 0];
else
    %if wrench rotation exists
    rotation_w_array_x = zeros(size(discrete_rot_angle_w_p, 1),4); %preallocating for speed
    rotation_w_array_y = zeros(size(discrete_rot_angle_w_p, 1),4); %preallocating for speed
    for i = 1 : size(discrete_rot_angle_w_p, 1) %K:calculate each of the rotation
        rotation_w_array_x(i, :) = [1 0 0 discrete_rot_angle_w_p(i)]; % a rotation of every 'discret angle' radians around the y-axis
        rotation_w_array_y(i, :) = [0 1 0 discrete_rot_angle_w_p(i)]; % a rotation of every 'discret angle' radians around the y-axis
    end
end

f_g = 0; % tbd Gewichtskraft implementieren wenn Gewicht bekannt

%% Parameter zur Arbeitsraum Berechnung
f_min = 5;
f_max = 36; % fmax berechnet: 2* 183 / 10 = 36, 6 %Motor 
limit.lower = (1/2 * (f_max - f_min)) ; %upper limit for improve closed-form solution (eq. 3.6 Pott book)
limit.upper = (1/2 * sqrt(noC) * (f_max - f_min)); %lower limit for improved closed form (eq. 3.6 Pott book)
% f_M = ones(noC,1); %preallocating for speed
counter_analysis = 1; %tbd counter logik ändern!!!!

coordinate.x = 0; %step size in x-direction
coordinate.y = 0; %step size in y-direction
coordinate.z = (grid.z_min : grid_delta: grid.z_max)'; %step size in z-direction

f_directions = ["x","y"]; %define the f_x and f_y wrench direction. 
workspace_logical = ~ones(length(coordinate.x), length(coordinate.y), length(coordinate.z)); %preallocating the variable for speed
workspace_logical_temp = ~ones(length(coordinate.x), length(coordinate.y), length(coordinate.z)); %preallocating the variable for speed




for f_xy=1:2
f_direction = f_directions(f_xy);

        for counter_r = 1 : size(rotation_array_x, 1)
        rotation_x = rotation_array_x(counter_r, :); %go through rotation array_x one by one
        rot_name = rotation_array_values_x(counter_r); %necessary to save the path name for figure automatically  

            for counter_z = 1: size(rotation_array_z,1)
%                 rotation_z = rotation_array_z(counter_z,:);
                rotation_z = rotation_array_xy(counter_z,:);
                rotation_axis=rot_axis(counter_z,:);
            
            if w_p_x == 0
             for counter_w = 1 %This is the only diff 
                rotation_w_p.x = rotation_w_array_x(counter_w, :);
                rotation_w_p.y = rotation_w_array_y(counter_w, :);
                [workspace_logical,  b_rot_xz, POI_rot,middle_rod] = Arbeitsraum_khaw(a, b, f_min, f_max, noC, rotation_x, rotation_z, rotation_axis, rot_name, w_p_x, w_p_t, rotation_w_p, workspace_logical, pulley_kin, R_A, rot_angle_A, coordinate, limit, f_direction);
             end
            else  %for w_p ~= 0 
                for counter_w = 1 : size(rotation_w_array, 1) %for w_p ~= 0 
                rotation_w_p.x = rotation_w_array_x(counter_w, :);
                rotation_w_p.y = rotation_w_array_y(counter_w, :);
                [workspace_logical,  b_rot_xz, POI_rot,middle_rod] = Arbeitsraum_khaw(a, b, f_min, f_max, noC, rotation_x, rotation_z, w_p_x, w_p_t, rotation_w_p, workspace_logical, pulley_kin, R_A, rot_angle_A, coordinate, limit, f_direction);
                end
            end

        %finalen Arbeitsraum bestimmen und darstellen
        counter_analysis = counter_analysis + 1;
        [workspace_logical, workspace_adapt_pointwise__trans] = Arbeitsraum_Verarbeitung_KHAW(a, b, grid_n,  w_p_x, w_p_t, f_g, counter_analysis ,rot_name, coordinate, workspace_logical,  b_rot_xz, noC, POI_rot,middle_rod);
   
            end
        end


end

toc














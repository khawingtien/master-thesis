%Maincode
% close all
clear 
clc
tic %start Stopwatch timer

%% !!!!!!! choose before run !!!!!!!
%pulley kinematic berücksichtigen JA/NEIN
%  pulley_kin = "yes";
pulley_kin = "no";
%% Parameter des Seilroboters laden
% Ebener Roboter mit 4 Seilen
% Definiere proximale Ankerpunkte Rahmen [x; y]


 %% pulley
global loc_winch
loc_winch = [-415, -415, 415,  415; -25,  25, 25, -25];
global rad_winch 
rad_winch = 5;
global rad_pulley
rad_pulley = 9.37+0.5;
global MP_pulley 
MP_pulley = [-331, -331, 331,  331; -331,  331, 331, -331];

if pulley_kin == 'yes'
    %tbd !!! evtl. auch unten in loop
    [Lfill, Rfill, tang_pulley_outer, R_A, rot_angle_A] = BerechnungTangentenPulley(loc_winch, rad_winch, rad_pulley, MP_pulley);
    a = tang_pulley_outer;

elseif pulley_kin == 'no'

ax = 0.230; %in m
ay = 0.230; %in m 
az = 0.102; %in m
[a] = SetupParameter(ax,ay,az);

%% Standardparameter
% Anzahl der Seile = number of cable 

noC = length(a);

R_A = 1; %just for input, is not in use 
rot_angle_A = 1; %just for input, is not in use  
end

% [ws_bowl_mat] = ws_position_bowl();
% 
% for bowl_index = 1: length(ws_bowl_mat)
%     ws_position = ws_bowl_mat(:,bowl_index);


%Define max and min of grid in all direction
grid.x_max = 300; %mm %largest length in x direction
grid.y_max = 300;
grid.z_max = 0;

grid.x_min = -300; %mm %smallest length in x direction
grid.y_min = -300;
grid.z_min = -620;

% grid_n = 20;  %Anzahl der Unterteilungen in X-Richtung
grid_n = 20;  %Anzahl der Unterteilungen in X-Richtung

%Definiere Grid                      
grid_delta = (grid.x_max - grid.x_min) / grid_n;  %step size in x-direction in mm %Gitterabstand von X-Richtung (Y- & Z-Richtung auch in diesem Abstand)

%% Definiere distale Ankerpunkte Plattform [x; y]
% hier werden verschiedene Konfigurationen b1, b2... betrachtet und in
% b_cell gespeichert
b_cell = endeffektor2();

%% Definiere zu untersuchende Rotationen des Endeffektors um die z-Achse
%  rotation_array_values = [-45;-40;-35;-30;-25;-20;-15;-10;-8;-6;-4;-2;0]; %13 times rotation angle
rotation_array_values = [0];
rotation_array = zeros(length(rotation_array_values),4); %preallocationg for speed

% rotation_array_values = [0;20;40;60;80];
for i = 1 : size(rotation_array_values, 1)
    rotation_array(i, :) = [1 0 0 ((pi/180) * rotation_array_values(i))];
end

%% Definiere zu untersuchende Lasten in bestimmte Raumrichtungen definiert durch rotation_w_p
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
counter_analysis = 1; %tbd counter logik ändern!!!!

%% Analyse Arbeitsraum
% untersucht werden verschiedene b's und Rotationen bei verschiedenen wrenches
%Setup Arbeitsraum Matrix
%Dimension 1: x-Koordinate der zu prüfenden Position
%Dimension 2: y-Koordinate der zu prüfenden Position
% 0 = Position nicht in const. orientation workspace enthalten
% 1 = enthalten

coordinate.x = (grid.x_min : grid_delta: grid.x_max)'; %step size in x-direction
coordinate.y = (grid.y_min : grid_delta: grid.y_max)'; %step size in y-direction
coordinate.z = (grid.z_min : grid_delta: grid.z_max)'; %step size in z-direction

%analysis for different parameter, will be saved in a Zeilenvektor. 
analysis = zeros(1, 12); %preallocating the variable for speed
C = cell(2,1); %predefine for speed
f_directions = ["x","y"]; %define the f_x and f_y wrench direction. 
figure %open a figure before the for-loop, so that x- and y-plane can be plotted on the same figure 


for ws_position_bowl = ws_bowl_mat(:,bowl_index)

    for f_xy=1:2 
    f_direction = f_directions(f_xy);

    % Calculation for workspace logical
    for counter_b = 1 : size(b_cell, 1) %counter for endeffector design type (line form, square form...)
    b = b_cell{counter_b, 1};
    b_name = counter_b; %necessary to save the path name for figure automatically 
        for counter_r = 1 : size(rotation_array, 1)
        rotation = rotation_array(counter_r, :); %go through rotation array one by one
        rot_name = rotation_array_values(counter_r); %necessary to save the path name for figure automatically
        
        %workspace_logical: in der 2. Dimension wird der Arbeitsraum für
        %jedes w_p gespeichert, anschließend mit 1. Dimension abgeglichen
        %und die Übereinstimmung in 1. Dimension gespeichert. Loop
        workspace_logical = ones(length(coordinate.x), length(coordinate.y), length(coordinate.z)); %preallocating the variable for speed
    
            if w_p_x == 0
             for counter_w = 1 %This is the only diff 
                rotation_w_p.x = rotation_w_array_x(counter_w, :);
                rotation_w_p.y = rotation_w_array_y(counter_w, :);
                [workspace_logical, R] = Arbeitsraum_khaw(a, b, f_min, f_max, noC, rotation, w_p_x, w_p_t, rotation_w_p, workspace_logical, pulley_kin, rad_pulley, R_A, rot_angle_A, coordinate, limit, f_direction,ws_position_bowl);
             end
            else  %for w_p ~= 0 
                for counter_w = 1 : size(rotation_w_array, 1) %for w_p ~= 0 
                rotation_w_p.x = rotation_w_array_x(counter_w, :);
                rotation_w_p.y = rotation_w_array_y(counter_w, :);
                [workspace_logical, R] = Arbeitsraum_khaw(a, b, f_min, f_max, noC, rotation, w_p_x, w_p_t, rotation_w_p, workspace_logical, pulley_kin, rad_pulley, R_A, rot_angle_A, coordinate, limit, f_direction,ws_position_bowl);
                end
            end

        %finalen Arbeitsraum bestimmen und darstellen
        counter_analysis = counter_analysis +   1;
        [analysis, workspace_logical, workspace_adapt_pointwise] = Arbeitsraum_Verarbeitung_KHAW(a, b, grid_n, b_name,  w_p_x, w_p_t, f_g, counter_analysis ,rot_name, analysis, coordinate, workspace_logical, R, noC);
       
        %%Save workspace in cell array  
        C(counter_r,2) = {workspace_adapt_pointwise};     
        end
    end
    %%Save workspace in cell array    
    C(f_xy,1) = {C};
    end
end

%Sheet : analysis
% excel_save = "1R2T_Rahmen1_%d_%d_%d_%d_%d_%d_%d.xlsx";
% excel_name = sprintf(excel_save, f_min, f_max, w_p, f_g, grid_n, grid_deg, w_p_t);
% writematrix(analysis, excel_name);
toc














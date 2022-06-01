close all
clear all
clc
tic %start Stopwatch timer

%% !!!!!!! choose before run !!!!!!!
%pulley kinematic berücksichtigen JA/NEIN
%  pulley_kin = "yes";
pulley_kin = "no";
%% Parameter des Seilroboters laden
% Ebener Roboter mit 4 Seilen
% Definiere proximale Ankerpunkte Rahmen [x; y]
global f_min 
f_min = 5; 
global f_max
f_max = 36;
global grid_length
grid_length = 662; %mm %Abstand zwischen Ankerpunkten am Rahmen
global grid_n
grid_n = 33;  %Anzahl der Unterteilungen
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
    a = [0   350  -250 -350 350 350  -350;  %x in mm 
         450 0   -450  450  450 -450 -450 ;  %y in mm 
         250 300  250  250  300 300  250 ]; %z in mm
    R_A = 1; %just for input, is not in use 
    rot_angle_A = 1; %just for input, is not in use     
end

%% Definiere distale Ankerpunkte Plattform [x; y]
% hier werden verschiedene Konfigurationen b1, b2... betrachtet und in
% b_cell gespeichert
b_cell = endeffektor2();

%% Definiere zu untersuchende Rotationen des Endeffektors um die z-Achse
% rotation_array_values = [-45;-40;-35;-30;-25;-20;-15;-10;-8;-6;-4;-2;0]; %13 times rotation angle
rotation_array_values = [0];
for i = 1 : size(rotation_array_values, 1)
    rotation_array(i, :) = [0 0 1 ((pi/180) * rotation_array_values(i))];
end

%% Definiere zu untersuchende Lasten in bestimmte Raumrichtungen definiert durch rotation_w_p
w_p = 0; %dieser Wert wird in berechnungSeilkraftverteilung in den wrench Vektor als x-Koordinate eingesetzt, y=0, T=0 (Feedback Kraft in alle Richtung) 
w_p_t = 0; %Torque (Feedback Kraft in Rotation)
grid_deg = 9; % rotatorische Auflösung
discrete_rot_angle_w_p = transpose(linspace(0, 2*pi, grid_deg)); %(x1, x2, n) n Punkte zwischen x1 und x2
rotation_w_array = zeros(size(discrete_rot_angle_w_p, 1), 4);
if w_p == 0
    rotation_w_array = [0, 0, 1, 0]; %Euler Winkel (x,y,z, Winkel), a rotation of 0 radians around the z-axis
else
    for i = 1 : size(discrete_rot_angle_w_p, 1) %K:calculate each of the rotation
        rotation_w_array(i, :) = [0 0 1 discrete_rot_angle_w_p(i)]; % a rotation of every 'discret angle' radians around the z-axis
    end
end

f_g = 0; % tbd Gewichtskraft implementieren wenn Gewicht bekannt
%% Standardparameter
% Basispunkte Roboter
size_a = size(a);

% Anzahl der Seile = number of cable 
global noC
noC = size_a(2);

%% Parameter zur Arbeitsraum Berechnung
% Definiere Grid                      
grid_delta = grid_length / grid_n;       %Gitterabstand

t = linspace(0,10,100); % 0 bis 10 Sekunden in 100 Schritten
steps = length(t);
loesung_closed_form = zeros(noC,steps); %tbd check check if needed
f = zeros(noC,steps);
f_min = 5;
f_max = 36; % fmax berechnet: 2* 183 / 10 = 36, 6 %Motor 

counter_analysis = 1; %tbd counter logik ändern!!!!

%% Analyse Arbeitsraum
% untersucht werden verschiedene b's und Rotationen bei verschiedenen wrenches
%Setup Arbeitsraum Matrix
%Dimension 1: x-Koordinate der zu prüfenden Position
%Dimension 2: y-Koordinate der zu prüfenden Position
% 0 = Position nicht in const. orientation workspace enthalten
% 1 = enthalten

coordinate.x = [grid_length/2 : -grid_delta: -grid_length/2]'; %x,y,z dimension should be the same 
coordinate.y = [grid_length/2 : -grid_delta: -grid_length/2]';
coordinate.z = [grid_length/2 : -grid_delta: -grid_length/2]'; %KHAW

%Define Workspace in a matrix with column1=x, column2=y and column3=z
workspace = [coordinate.x coordinate.y coordinate.z] ;
%analysis for different parameter, will be saved in a Zeilenvektor. preallocating the variable for speed
analysis = zeros(1, 7);

%% Calculation for workspace logical
for counter_b = 1 : size(b_cell, 1) %counter for endeffector design type (line form, square form...)
    b = b_cell{counter_b, 1};
    b_name = counter_b; %extra 
    for counter_r = 1 : size(rotation_array, 1)
        rotation = rotation_array(counter_r, :); %go through rotation array one by one
        rot_name = counter_r; %extra
        
        %workspace_logical: in der 2. Dimension wird der Arbeitsraum für
        %jedes w_p gespeichert, anschließend mit 1. Dimension abgeglichen
        %und die Übereinstimmung in 1. Dimension gespeichert. Loop
        workspace_logical = ones(grid_n + 1, grid_n + 1, grid_n + 1); %preallocating the variable for speed
        workspace_logical_temp = ones(grid_n + 1, grid_n + 1, grid_n + 1); %preallocating the variable for speed
        for counter_w = 1 : size(rotation_w_array, 1)
            rotation_w_p = rotation_w_array(counter_w, :);
            [workspace_logical, R] = Arbeitsraum_khaw(a, b, f_min, f_max, grid_n, rotation, w_p, w_p_t, rotation_w_p, workspace, workspace_logical, pulley_kin, rad_pulley, R_A, rot_angle_A,coordinate);
        end
        %finalen Arbeitsraum bestimmen und darstellen
        [analysis, workspace_logical, workspace_adapt_pointwise] = Arbeitsraum_Verarbeitung_KHAW(a, b, grid_n, b_name,  w_p, w_p_t, f_g, counter_analysis, rot_name, analysis, workspace, workspace_logical, R, grid_deg);
        counter_analysis = counter_analysis + 1; 
    end
end
%Sheet : analysis
% excel_save = "1R2T_Rahmen1_%d_%d_%d_%d_%d_%d_%d.xlsx";
% excel_name = sprintf(excel_save, f_min, f_max, w_p, f_g, grid_n, grid_deg, w_p_t);
% writematrix(analysis, excel_name);
toc














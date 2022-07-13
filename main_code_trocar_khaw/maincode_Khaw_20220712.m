% close all
% clear 
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
    
%     a = [0   350  -250 -350 350 350  -350;  %x in mm 
%          450 0   -450  450  450 -450 -450 ;  %y in mm 
%          250 300  250  250  300 300  250 ]; %z in mm

%% for 6 Cable triangular winch position (1)too small (500mm/700mm)02b0
%     a = [-250 250  250  -250 250 250  0 0;  %x in mm (change to 350mm for l = 700mm)
%          250 250   -250  250  250 -250 0 0;  %y in mm 
%          300 300  300  250  250 250  0  0]; %z in mm

%% for 6 Cable triangular winch position (2) too big (length = 800mm)02b1
% ax = 0.4; 
% ay = 0.4;
% az = 0.6;
%     a = [-ax  ax ax -ax ax ax  0 0 ;  %x in m 
%          ay ay   -ay  ay ay -ay 0 0;  %y in m
%          az az  az  0.5  0.5 0.5 0  0]; %z in m
%  a = a.*1000; %a in mm   

%% for 6 Cable parallel triangle winch position (700mm)02b2
%     a = [-120 350  -250  -350 350 350 -350  0 ;  %x in mm 
%          450  0   -450  450  450  -450 -450 0;  %y in mm 
%          250 300  250  250  300  300  250  0]; %z in mm
     
%% for 6 Cable hexagone (600mm)02c 
%    a = [300 -225  -225  225 225 -300 0  0 ;  %x in mm 
%         0  -300   300  -300  300  0 0 0;  %y in mm 
%         300 300  300  250  250  250   0  0]; %z in mm
     
%% for 8 cable standard configuration (WireX landing page)  
% a = [-20   20   20  -20  -20  20  20 -20;  %x in mm 
%      15    15  -15  -15  15   15  -15 -15;  %y in mm 
%      20    20   20   20  0     0   0   0];  %z in mm
% a = a.*100; %in mm 

%% for 8 cable falcon configuration (1)  
% a = [-0.25   0.25  0.25   -0.25  -0.25  0.25  0.25 -0.25;  %x in m 
%      0.1875    0.1875  -0.1875  -0.1875  0.1875   0.1875  -0.1875 -0.1875;  %y in m 
%      0.125    0.125   0.125   0.125  0.0625     0.0625   0.0625   0.0625];  %z in m
% a= a.*10; %in mm 


% for 6 cable Hexagon confuguration (old)
% a = [-0.20   0.2   0     0     0.2   -0.2  0  0;  %x in m 
%      0.15    0.15  -0.15 0.15  -0.15 -0.15  0 0;  %y in m 
%      0.20    0.2   0.2   0     0     0    0  0];  %z in m
% a= a.*1000; %in mm 


%% for 8-wires_robot.py cable falcon configuration  (Artur)
% a = [2.7   2.7   -2.7  -2.7  2.7  2.7  -2.7   -2.7;  %x in m 
%      2.7   -2.7  -2.7  2.7   2.7  -2.7  -2.7  2.7;  %y in m 
%      2.7   2.7   2.7   2.7  -2.7  -2.7  -2.7  -2.7];  %z in m
% a= a.*1000; %in mm 

%% for 8 cable falcon configuration (2) (02a)  
% ax = 0.230; %x in m
% ay = 0.230; %y in m 
% az = 0.102; %z in m
% a = [ax   ax   -ax    -ax   ax   ax    -ax   -ax ;   
%      ay   -ay  -ay  ay   ay  -ay  -ay  ay;  
%      az    az   az   az   -az  -az   -az    -az ]; 
% a= a.*1000; %in mm 

%% for 6 cable falcon configuration (20220704)
% ax = 0.230; %x in m
% ay = 0.230; %y in m 
% az = 0.520; %z in m
% a = [0   ax   -ax    0   0   ax    -ax   0 ;   
%      ay   -ay  -ay  0   ay  -ay  -ay  0;  %y in m 
%      az    az   az   0   -az  -az   -az    0 ]; 
% a= a.*1000; %in mm 

ax = 0.230;
ay = 0.230; 
az = 0.102;
[a] = SetupParameter(ax,ay,az);

R_A = 1; %just for input, is not in use 
rot_angle_A = 1; %just for input, is not in use  
end

    
%Define max and min of grid in all direction
grid.x_max = max(a(1,:)); %mm %largest length in x direction
grid.y_max = max(a(2,:));
grid.z_max = max(a(3,:));

grid.x_min = min(a(1,:)); %mm %smallest length in x direction
grid.y_min = min(a(2,:));
grid.z_min = min(a(3,:));

% grid_n = 20;  %Anzahl der Unterteilungen in X-Richtung
grid_n = 23;  %Anzahl der Unterteilungen in X-Richtung

%Definiere Grid                      
grid_delta = (grid.x_max - grid.x_min)  / grid_n;  %step size in x-direction in mm %Gitterabstand von X-Richtung (Y- & Z-Richtung auch in diesem Abstand)

%% Definiere distale Ankerpunkte Plattform [x; y]
% hier werden verschiedene Konfigurationen b1, b2... betrachtet und in
% b_cell gespeichert
b_cell = endeffektor2();

%% Definiere zu untersuchende Rotationen des Endeffektors um die z-Achse
%  rotation_array_values = [-45;-40;-35;-30;-25;-20;-15;-10;-8;-6;-4;-2;0]; %13 times rotation angle
 rotation_array_values = 0;
for i = 1 : size(rotation_array_values, 1)
    rotation_array(i, :) = [0 0 1 ((pi/180) * rotation_array_values(i))];
end

%% Definiere zu untersuchende Lasten in bestimmte Raumrichtungen definiert durch rotation_w_p
w_p = 0; %dieser Wert wird in berechnungSeilkraftverteilung in den wrench Vektor als x-Koordinate eingesetzt, y=0, T=0 (Feedback Kraft in alle Richtung) 
w_p_t = 0; %Torque (Feedback Kraft in Rotation)%wrench in torque
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
% Anzahl der Seile = number of cable 
global noC
noC = length(a);

%% Parameter zur Arbeitsraum Berechnung
t = linspace(0,10,100); % 0 bis 10 Sekunden in 100 Schritten
steps = length(t);
loesung_closed_form = zeros(noC,steps); %tbd check check if needed
f = zeros(noC,steps);
f_min = 5;
f_max = 36; % fmax berechnet: 2* 183 / 10 = 36, 6 %Motor 

counter_analysis = 1; %tbd counter logik ändern!!!!
% counter_analysis_proj = 1;

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

%% Calculation for workspace logical
for counter_b = 1 : size(b_cell, 1) %counter for endeffector design type (line form, square form...)
    b = b_cell{counter_b, 1};
    b_name = counter_b; %necessary to save the path name for figure automatically 
    for counter_r = 1 : size(rotation_array, 1)
        rotation = rotation_array(counter_r, :); %go through rotation array one by one
        rot_name = counter_r; %necessary to save the path name for figure automatically
        
        %workspace_logical: in der 2. Dimension wird der Arbeitsraum für
        %jedes w_p gespeichert, anschließend mit 1. Dimension abgeglichen
        %und die Übereinstimmung in 1. Dimension gespeichert. Loop
        workspace_logical = ones(length(coordinate.x), length(coordinate.y), length(coordinate.z)); %preallocating the variable for speed

        for counter_w = 1 : size(rotation_w_array, 1)
            rotation_w_p = rotation_w_array(counter_w, :);
            [workspace_logical, R] = Arbeitsraum_khaw(a, b, f_min, f_max, grid_n, rotation, w_p, w_p_t, rotation_w_p, workspace_logical, pulley_kin, rad_pulley, R_A, rot_angle_A, coordinate);
        end

        %finalen Arbeitsraum bestimmen und darstellen
        counter_analysis = counter_analysis +   1;
        [analysis, workspace_logical, workspace_adapt_pointwise] = Arbeitsraum_Verarbeitung_KHAW(a, b, grid_n, b_name,  w_p, w_p_t, f_g, counter_analysis ,rot_name, analysis, coordinate, workspace_logical, R, grid_deg);
       
   end
end
%Sheet : analysis
% excel_save = "1R2T_Rahmen1_%d_%d_%d_%d_%d_%d_%d.xlsx";
% excel_name = sprintf(excel_save, f_min, f_max, w_p, f_g, grid_n, grid_deg, w_p_t);
% writematrix(analysis, excel_name);
toc














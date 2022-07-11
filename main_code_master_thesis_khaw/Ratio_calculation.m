%Berechnung von Verhältnis aus Rahmengröße zu Arbeitsraum

%Volume of Cylinder (ROI)
r = 150; %in mm 
h = 200; %in mm
vol_cylinder = pi*r^2*h; 

%%Configuration 1 (Laporostick)
vol_1 = 500*500*500;
ratio_conf_1 = vol_1/vol_cylinder; 

%%Configuration 2a 
vol_2a = 460*460*204;
ratio_conf_2a = vol_2a/vol_cylinder; 

%%Configuration 2b 
b = 1131*sqrt((800^2-(1131/2)^2)); %b*h (h calculate with pythagorean theorem) 
h_triangle = 100; % in mm 
vol_2b = 0.5*b*h_triangle;
ratio_conf_2b = vol_2b/vol_cylinder; 

%%Configuration 2d (new) 
vol_2d = 700*900*50;
ratio_conf_2d = vol_2d/vol_cylinder; 

%%Configuration 2c  
s =  600*cos(deg2rad(30)) ; %in mm
h_hexagon = 50; % in mm 
vol_2c = 3/2 * sqrt(3) * s^2 *h_hexagon;
ratio_conf_2d = vol_2c/vol_cylinder; 

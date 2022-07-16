function [b_cell] = endeffektor2()
%% Linienförmiger Endeffektor - Seil gerade
% 5cm Linie horizontal
b1 = [-25, -25, 25, 25;
    0, 0, 0, 0];
% 10cm Linie horizontal
b2 = [-50, -50, 50, 50;
    0, 0, 0, 0];
% 15cm Linie horizontal
b3 = [-75, -75, 75, 75;
    0, 0, 0, 0];

%% Rechteckiger Endeffektor - Seil gerade
% 5cm Quadrat
b4 = [-25, -25, 25, 25;
    -25, 25, 25, -25];
% 10cm Quadrat
b5 = [-50, -50, 50, 50;
    -50, 50, 50, -50];
% 15cm Quadrat
b6 = [-75, -75, 75, 75;
    -75, 75, 75, -75];
% 5 x 10 cm  (x y)
b7 = [-25, -25, 25, 25;
    -50, 50, 50, -50];
% 5 x 15 cm  (x y)
b8 = [-25, -25, 25, 25;
    -75, 75, 75, -75];
% 10 x 15 cm  (x y)
b9 = [-50, -50, 50, 50;
    -75, 75, 75, -75];

%% Dreieckiger Endeffektor
% 5 x 0 cm (x y)
b10 = [0, -25, 25, 0;
    0, 0, 0, 0];
% 10 x 0 cm (x y)
b11 = [0, -50, 50, 0;
    0, 0, 0, 0];
% 15 x 0 cm (x y)
b12 = [0, -75, 75, 0;
    0, 0, 0, 0];
% 5 x 5 cm (x y)
b13 = [0, -25, 25, 0;
    -25, 25, 25, -25];
% 10 x 5 cm (x y)
b14 = [0, -50, 50, 0;
    -25, 25, 25, -25];
% 15 x 5 cm (x y)
b15 = [0, -75, 75, 0;
    -25, 25, 25, -25];
% 10 x 10 cm (x y)
b16 = [0, -50, 50, 0;
    -50, 50, 50, -50];
% 15 x 10 cm (x y)
b17 = [0, -75, 75, 0;
    -50, 50, 50, -50];
% 15 x 15 cm (x y)
b18 = [0, -75, 75, 0;
    -75, 75, 75, -75];

%% Punktförmiger Endeffektor
% b19 = [0   0  0  0    0   0;
%        0   0  0  0    0   0;
%        30 30 30 -30 -30 -30];

b20 = [0   0  0    0   0   0    0  0;
       0   0  0    -1  -1    0    0 0;
       550 550 550 -2  -2    -2   0 0];

%% for 6 Cable triangular winch position (2)
b27 = [0   0  0    0   0   0    0  0;
       0   0  0    -0.04  -0.04    0    0 0;
       2.2 2.2 2.2 -0.08  -0.08     -0.08    0 0];
b27 = b27.*1000; 
%% for 8 cable square endeffector (WireX Landing Page)
b21 = [-0.6  0.6  0.6    -0.6   -0.6   0.6    0.6  -0.6;
       0.6   0.6  -0.6   -0.6   0.6    0.6    -0.6  -0.6;
       0      0     0     0       0     0      0        0];
b21 = b21.*100;

%% for 8 cable falcon endeffector (1)
bz = 0.375;
b22 = [-0.015  0.015  0.015    -0.015   -0.015   0.015    0.015   -0.015;
       0.015   0.015  -0.015   -0.015   0.015    0.015    -0.015  -0.015;
       bz+0.1  bz+0.1   bz+0.1     bz+0.1   -bz-0.1    -bz-0.1   -bz-0.1  -bz-0.1 ];
b22 = b22.*10;


%% for 6 cable hexagone  configuration (old)
b23 = [-0.06  0.06  0.01    -0.01   0.06   -0.04    0  0;
       0.06   0.06  -0.06   -0.06   -0.04   -0.06    0  0;
       1      1      1      0        0      0       0   0 ];
b23 = b23.*1000;

%% for 8-wires_robot.py cable falcon configuration  (Artur)  
b24 = [0.2  0.2     -0.2    -0.2    0.2   0.2   -0.2   -0.2 ;
       0.2   -0.2   -0.2    0.2   0.2   -0.2    -0.2   0.2 ;
       3.55  3.55  3.55   3.55   -3.05  -3.05  -3.05  -3.05 ];
 b24 = b24./10;

%% for 8 cable falcon endeffector (2) (02a)
%a. rod length = 320mm 
bx = 0.017; %in m
by = 0.017; %in m
bz = 0.16;  %in m
b25a = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       bz  bz   bz    bz   -bz    -bz   -bz  -bz];
b25a = b25a.*1000; %in mm

%b. rod length = 460mm
bx = 0.017; %in m
by = 0.017; %in m
bz = 0.23;  %in m
b25b = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       bz  bz   bz    bz   -bz    -bz   -bz  -bz];
b25b = b25b.*1000; %in mm

%c. rod length = 460mm
bx = 0.017; %in m
by = 0.017; %in m
bz = 0.25;  %in m
b25c = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       bz  bz   bz    bz   -bz    -bz   -bz  -bz];
b25c = b25c.*1000; %in mm

%d. rod length = 920mm
bx = 0.017; %in m
by = 0.017; %in m
bz = 0.46;  %in m
b25d = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       bz  bz   bz    bz   -bz    -bz   -bz  -bz];
b25d = b25d.*1000; %in mm

%e. rod length = 1000mm
bx = 0.017; %in m
by = 0.017; %in m
bz = 0.5;  %in m
b25e = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       bz  bz   bz    bz   -bz    -bz   -bz  -bz];
b25e = b25e.*1000; %in mm

%f. rod length = 1200mm
bx = 0.017; %in m
by = 0.017; %in m
bz = 0.6;  %in m
b25f = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       bz  bz   bz    bz   -bz    -bz   -bz  -bz];
b25f = b25f.*1000; %in mm
%% for 6 Cable triangular winch position (2) too big (length = 800mm)02b1
bx = 0; %in m
by = -0.02; %in m
bz = 1.1;  %in m
b26 = [bx  bx  -bx    0  bx   bx    -bx   0;
       0  0  0  -by   -by    0    0  0;
       bz  bz   bz    -0.04   -0.04    -0.04   0  0];
b26 = b26.*1000; %in mm

%% for 6 Cable parallel triangle winch position (700mm)02b2
b28 = [0  0  0      0  0   0   0   0;
       0  0  0      10 10  -10 -10  0;
       500 500 500  0   0  0   0   0];
   
%% for 6 Cable hexagone (600mm)02c
b29 = [0  0  0      0  0   0   0   0;
       0  0  0      0 0  0 0  0;
       550 550 550 -200   -200   -200  0   0];

%%Trocar test
bx = 0.017; %in m
by = 0.017; %in m
bz_up = 0.4;  %in m
bz_down = 0.6;
b30 = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       bz_up  bz_up   bz_up    bz_up   -bz_down    -bz_down   -bz_down  -bz_down];
b30 = b30.*1000; %in mm

%  b_cell = {b1; b2; b3; b4; b5; b6; b7; b8; b9; b10; b11;
%      b12; b13; b14; b15; b16; b17; b18; b19};
b_cell = {b25e}; 
 end 

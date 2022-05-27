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
b19 = [0, 0, 0, 0;
    0, 0, 0, 0;
    0,0,0,0];

%  b_cell = {b1; b2; b3; b4; b5; b6; b7; b8; b9; b10; b11;
%      b12; b13; b14; b15; b16; b17; b18; b19};
b_cell = {b19} %%KHAW
 end 
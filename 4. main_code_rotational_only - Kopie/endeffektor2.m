function [b_cell] = endeffektor2()

%%Trocar test
bx = 0.017; %in m
by = 0.017; %in m
bz_up = 0.3;  %in m
bz_down = 0.3;
b30 = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       bz_up  bz_up   bz_up    bz_up   -bz_down    -bz_down   -bz_down  -bz_down];
b30 = b30.*1000; %in mm

%  b_cell = {b1; b2; b3; b4; b5; b6; b7; b8; b9; b10; b11;
%      b12; b13; b14; b15; b16; b17; b18; b19};
b_cell = {b30}; 
 end 

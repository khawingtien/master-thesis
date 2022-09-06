function [b_cell] = endeffektor2()

%% standard parameter 
bx = 0.017; %in m
by = 0.017; %in m

%% DEFINE Rod length parameter here
% bz = 2000:100:3000; %in mm 
bz = 0.6; %in m standard rod length 
% bz = 0.1:0.1:1.0;


b_cell = cell(length(bz),1);

for i = 1:length(bz)
    bz_value = bz(i);

    b = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
           by  -by  -by   by   by    -by    -by  by;
           bz_value/2  bz_value/2   bz_value/2    bz_value/2   -bz_value/2 -bz_value/2   -bz_value/2  -bz_value/2];
    b = b*1000; %in mm
    

b_cell{i} = b; 
 end 

function [b_cell] = endeffektor2(bz)

%% standard parameter 
bx = 0.005; %in m
by = 0.005; %in m

%% DEFINE Rod length parameter here
% bz = 0.6; %in m standard rod length 
% bz = 0.2:0.3:2.6;


b_cell = cell(length(bz),1);

for i = 1:length(bz)
    bz_value = bz(i);

    b = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
           by  -by  -by   by   by    -by    -by  by;
           bz_value/2  bz_value/2   bz_value/2    bz_value/2   -bz_value/2 -bz_value/2   -bz_value/2  -bz_value/2];
    b = b*1000; %in mm
    

b_cell{i} = b; 
 end 

function [b] = Setup_Parameter_Endeffector_khaw(bz_value) %input in mm 

%% standard parameter 
bx = 0.005*1000; %in mm
by = 0.005*1000; %in mm

%% DEFINE Rod length parameter here
% bz = 0.6; %in m standard rod length 


    b = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
         by  -by  -by   by   by    -by    -by  by;
         bz_value/2  bz_value/2   bz_value/2    bz_value/2   -bz_value/2 -bz_value/2   -bz_value/2  -bz_value/2];
    

end 

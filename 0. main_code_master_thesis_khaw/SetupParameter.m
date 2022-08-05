function [a] = SetupParameter(ax,ay,az) %in m
%%Setup of Dimension of Cable Driven Robot 

%% for 8 cable falcon configuration (2) (02a)  
a = [ax   ax   -ax    -ax   ax   ax    -ax   -ax ;   
     ay   -ay  -ay  ay   ay  -ay  -ay  ay;  
     az    az   az   az   -az  -az   -az    -az ]; 
a= a.*1000; %in mm 
end
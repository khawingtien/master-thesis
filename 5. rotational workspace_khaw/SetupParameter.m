function [a_cell] = SetupParameter() 
%%Setup of Dimension of Cable Driven Robot 

%% Define parameter here 
% ax = 0.2: 0.1: 0.7; %in [m] Aufbau max(0.56m) min(0.2m), sonst keine ws
ax = 0.23;

%% Standard parameter 
ay = 0.230; %in [m] Aufbau max(0.56m)
az = 0.06; %in [m] Aufbau max (0.26m) 

a_cell = cell(length(ax),1);

for i = 1:length(ax)
    ax_value = ax(i);

a = [ax_value   ax_value   -ax_value  -ax_value ax_value ax_value  -ax_value -ax_value ;   
     ay   -ay  -ay  ay   ay  -ay  -ay  ay;  
     az    az   az   az   -az  -az   -az    -az ]; 

a = a.*1000; %in [mm]

a_cell{i} = a; %in [mm]

end
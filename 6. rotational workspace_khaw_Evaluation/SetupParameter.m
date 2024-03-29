function [a_cell] = SetupParameter(ax,ay, az) 
%%Setup of Dimension of Cable Driven Robot 

%% Define parameter here 
% ax = 0.2: 0.1: 0.7; %in [m] Aufbau max(0.56m) min(0.2m), sonst keine ws
% ax = 0.1:0.1:0.6;
% ax = 0.23;

%% Standard parameter 
% ay = ax;
% ay = 0.60; %in [m] Aufbau max(0.56m)
% az = 0.06; %in [m] Aufbau max (0.26m) 

a_cell = cell(max(length(ax), length(az)),1); %choose the longer parameter, while the other would be only one 
ax_value = ax(1);
az_value = az(1);

    for i = 1:length(a_cell) 
        if length(ax)~=1 %it could only be one of the both condition (only 1 variable & 1 fix) 
            ax_value = ax(i);
        elseif length(az)~=1
            az_value = az(i);
        end
    
    a = [ax_value/2   ax_value/2   -ax_value/2  -ax_value/2 ax_value/2 ax_value/2  -ax_value/2 -ax_value/2 ;   
         ay/2   -ay/2  -ay/2  ay/2   ay/2  -ay/2  -ay/2  ay/2;  
         az_value/2    az_value/2   az_value/2   az_value/2   -az_value/2  -az_value/2   -az_value/2    -az_value/2 ]; 
    
    a = a.*1000; %in [mm]    
    a_cell{i} = a; %in [mm]

    end

end
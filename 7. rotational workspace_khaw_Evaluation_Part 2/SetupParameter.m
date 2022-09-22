function [a_cell] = SetupParameter(ax,ay,az)  %input in mm 
%%Setup of Dimension of Cable Driven Robot 

%% Standard parameter 

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
       
    a_cell{i} = a; %in [mm]

    end

end
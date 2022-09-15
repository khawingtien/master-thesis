function [a] = Setup_Parameter_khaw(ax_value,ay_value,az_value) %input in mm 
%%Setup of Dimension of Cable Driven Robot 


    a = [ax_value/2   ax_value/2   -ax_value/2  -ax_value/2 ax_value/2 ax_value/2  -ax_value/2 -ax_value/2 ;   
         ay_value/2   -ay_value/2  -ay_value/2  ay_value/2   ay_value/2  -ay_value/2  -ay_value/2  ay_value/2;  
         az_value/2    az_value/2   az_value/2   az_value/2   -az_value/2  -az_value/2   -az_value/2    -az_value/2 ]; 
     
    
end
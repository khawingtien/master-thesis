function [percentage] = Workspace_in_ROI(workspace_adapt_pointwise)
r = 150; %150mm in radius 
z_circle_min = -102; %minimum height
z_circle_max = 98; %maximum height

counter_circle = 0; %counter of point within ROI is 0 at first 
 for row = 1:length(workspace_adapt_pointwise)

   x_circle =  workspace_adapt_pointwise(row,1);
   y_circle =  workspace_adapt_pointwise(row,2);
   z_circle =  workspace_adapt_pointwise(row,3);
   
   %check whether the point within the circle diameter (300mm)
   if x_circle^2 + y_circle^2 <= r^2
       %check whether the point within the height (200mm)
       if z_circle_min <= z_circle && z_circle <= z_circle_max 
       counter_circle = counter_circle +1; 
       end 
   end 
 end
format long
percentage = counter_circle/ length(workspace_adapt_pointwise)*100; %calculate the percentage of ws within ROI
%%Display
formatSpec = "%e %s of workspace was in ROI ";
A1 = percentage;  
A2 = '%';
str = sprintf(formatSpec,A1,A2)
end
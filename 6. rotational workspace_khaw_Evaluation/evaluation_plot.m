function  evaluation_plot(I_vv_mat,rod_length, x_label)
%UNTITLED4 Summary of this function goes here

%   Detailed explanation goes here
% figure
% plot(rod_length,I_vv_mat,'k-o','LineWidth',3)
% xlabel('Rod Length [mm]')
% ylabel('Ivv')
% title('linear progression of Ivv')
% grid on 


figure
plot(rod_length,I_vv_mat,'k-o','LineWidth',3)
% xlabel(x_label,' [mm]')
ylabel('Ivv')
title('linear progression of Ivv')
grid on 

end
function  evaluation_plot(plot_x_axis,I_vv_mat, x_label,txt)
%input of plot_x_axis is different based on case.  

%plot the figure 
figure
plot(plot_x_axis,I_vv_mat,'k-o','LineWidth',3)
xlabel(x_label)
ylabel('Ivv')
title('linear progression of Ivv')
subtitle(txt)
grid on 

end
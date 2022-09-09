function  evaluation_plot(plot_x_axis,I_vv_mat,Volume_ws_mat,Volume_frame_mat,x_label,txt)
%input of plot_x_axis is different based on case.  

%% Define input arguement here 
x = plot_x_axis;
acc=I_vv_mat;
vel=Volume_ws_mat*1e-9; %in m3
pos=Volume_frame_mat*1e-9; %in m3

%% Sets of three inputs (no figure given)

linecolors={'r' [0 .5 0] 'b'};

h3i=plotNy(x,acc,1,...
    x,vel,2,...
    x,pos,3,...
    'Linewidth',1,...
    'YAxisLabels',{'[-]' '[m^3]' '[m^3]'},...
    'XAxisLabel', '',...
    'TitleStr','Evaluation of workspace',...
    'LineColor',linecolors,...
    'FontSize',10,...
    'Fontname','TimesNewRoman',...
    'Grid','on',...
    'LegendString',{'Ivv [-]' 'Volume of workspace [m^3]' 'Volume of frame [m^3]'});


    %Change the axis colors to match the requested line colors
    for i=1:length(h3i.ax)
	    set(h3i.ax(i),'ycolor',linecolors{i});	
    end

subtitle(txt) 
xlabel(x_label) %overwrite the xlabel defined in function

end


%plot the figure 
% figure
% % yyaxis left
% plot(plot_x_axis,I_vv_mat,'k-o','LineWidth',3)
% xlabel(x_label)
% ylabel('Ivv')
% title('linear progression of Ivv')
% subtitle(txt)
% grid on 
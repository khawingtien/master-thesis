file_title = sprintf(filename,ax_value ,ay_value, az_value, b_value);

for k = 1
fig = figure(k);
path = 'D:\Masterarbeit\11_MATLAB_GIT\6. rotational workspace_khaw_Evaluation\figure\20220909';
filename = "rotational_workspace_%d_%d_%d_%d";
filename_total = sprintf(filename,ax_value ,ay_value, az_value, b_value);
saveas(figure(k),fullfile(path,filename_total),'png');
% print([destination,num2str(k),'.png']);
close(fig)
end 

filename = "rotational_workspace_%d_%d_%d_%d";
path = sprintf(filename, b_name, rot_name, w_p, w_p_t);
saveas(figure(counter_analysis), path, 'png');
close(figure(counter_analysis))



for counter = 1
filename = "rotational_workspace_%d_%d_%d_%d";
path = sprintf(filename,ax_value ,ay_value, az_value, b_value);
% saveas(figure(counter), path, 'png');
saveas(gca, fullfile(path, filename), 'jpeg');
close(figure(counter))
end


saveas(gca, fullfile(path, filename), 'jpeg');
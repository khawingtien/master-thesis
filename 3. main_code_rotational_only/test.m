figure
b_figure = R * b; %Rotation * base(end-effector)
b_5 = b_figure(:,1); %so that it close up the endeffector upper side
b_10 = b_figure(:,5); %so that it close up the endeffector lower side
b_figure_new = [b_figure(:,1:4) b_5 b_figure(:,5:8) b_10];
plot3(b_figure_new(1, :), b_figure_new(2, :),b_figure_new(3, :), 'x-k','LineWidth',2);
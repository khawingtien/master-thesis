

indices = find(isnan(workspace_trans_mat) == 1);
[row,col] = ind2sub(size(X),indices);

for ilength

[k, convexhull_volume] = convhull(workspace_trans_mat,'Simplify',true);
figure
trisurf(k,workspace_trans_mat(:,1),workspace_trans_mat(:,2),workspace_trans_mat(:,3),'FaceColor','b','Edgecolor','b')


str = ["w1" "w2" "w3" "w4" "w5" "w6" "w7" "w8"]; 
for i = 1 : noC
    plot3([a_adapt(1, i) b(1, i)], [a_adapt(2, i) b_figure(2, i)], [a_adapt(3, i) b_figure(3, i)],'--r');
    text(a_adapt(1, i), a_adapt(2,i), a_adapt(3,i), str(i)); %add the label on each cable 
end

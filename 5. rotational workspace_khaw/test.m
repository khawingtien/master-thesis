
indices = find(isnan(workspace_trans_mat_total) == 1);
workspace_trans_mat_total(indices) = 0; 

[k, convexhull_volume] = convhull(workspace_trans_mat_total,'Simplify',true);
figure
trisurf(k,workspace_trans_mat_total(:,1),workspace_trans_mat_total(:,2),workspace_trans_mat_total(:,3),'FaceColor','b','Edgecolor','b')



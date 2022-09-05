clear
load("20220905_ws_logical_wrench_cell.mat")
result=cell(size(ws_logical_cell_0_m1,1), size(ws_logical_cell_0_m1,2));
workspace_cell=cell(size(ws_logical_cell_0_m1,1), size(ws_logical_cell_0_m1,2));
workspace_cell_1_m1=cell(size(ws_logical_cell_0_m1,1), size(ws_logical_cell_0_m1,2));

for i=1:size(ws_logical_cell_0_m1,1)
    for j=1:size(ws_logical_cell_0_m1,2)
        result{i, j} = and(ws_logical_cell_0_m1{i, j}.logical, ws_logical_cell_1_0{i, j}.logical);
        [workspace_pointwise_trans] = ws_translation_khaw(result{i,j},coordinate,ws_logical_cell_0_m1{i, j}.POI);
        [workspace_pointwise_trans_1_m1] = ws_translation_khaw(ws_logical_cell_1_m1{i,j}.logical,coordinate,ws_logical_cell_0_m1{i, j}.POI);
        workspace_cell{i, j} = workspace_pointwise_trans;
        workspace_cell_1_m1{i, j} = workspace_pointwise_trans_1_m1;
    end
end

workspace_trans_mat = cat(1, workspace_cell{:,:});
workspace_trans_mat_1_m1 = cat(1, workspace_cell_1_m1{:,:});
figure

plot3(workspace_trans_mat(:,1),workspace_trans_mat(:,2),workspace_trans_mat(:,3),'.g') %symmetry 
hold on
%  plot3(workspace_trans_mat_1_m1(:,1),workspace_trans_mat_1_m1(:,2),workspace_trans_mat_1_m1(:,3),'.r')
%  unsymmetry
%  
hold on
axis equal

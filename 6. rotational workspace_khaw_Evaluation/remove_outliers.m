function workspace_trans_remove_OutL = remove_outliers(workspace_trans_mat)
%remove outlier in plot
workspace_trans_remove_OutL = workspace_trans_mat;
dist_mat = zeros(length(workspace_trans_mat),1);
P = workspace_trans_mat;

    for i= 1:length(workspace_trans_mat)
    PQ=P(i,:); %PQ = Query point (point to be checked) 
    [k,dist] = dsearchn(P(1:end ~=i,:),PQ); %search the neighbout without itself
    dist_mat(i,1) = dist; 
    end

%query point will be removed
[row] = find(dist_mat >18); 
workspace_trans_remove_OutL(row,:) = []; %indexing using outliers and setting them to NULL
end


% figure
% plot3(P(:,1),P(:,2),P(:,3),'ko')
% hold on
% plot3(PQ(:,1),PQ(:,2),PQ(:,3),'*g')
% hold on
% plot3(P(k,1),P(k,2),P(k,3),'*r')
% legend('Data Points','Query Points','Nearest Points','Location','sw')


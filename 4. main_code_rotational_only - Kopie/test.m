clear
clc

rP = [];
rP2 = [];
b =  [ 17    17   -17   -17    17    17   -17   -17
    17   -17   -17    17    17   -17   -17    17
   300   300   300   300  -300  -300  -300  -300];

figure 
for quar_z = 1:360
    for quar_x = 1:30
        quat = quaternion([quar_x, 0, quar_z],'eulerd','XYZ','point');
        for b_i = 1
            b_index = (b(:,b_i))';       
            rP_temp = (rotatepoint(quat,b_index))';
            rP = [rP rP_temp];%append rP
        end
        plot3(rP(1,:),rP(2,:),rP(3,:),'ro')
        hold on 
%     rP2 = [rP2 rP];
    end 
end


	%%Comparison 
   
	 rotation = [1 0 0 deg2rad(1)]; %rotation at x-axis
	 R_x = axang2rotm(rotation);
	 rotation_z = [0 0 1 deg2rad(1)]; %rotation at x-axis
	 R_z = axang2rotm(rotation_z);
 b_rot = R_z*(R_x * b); %rotation matrix 
plot3(b_rot(1,:),b_rot(2,:),b_rot(3,:),'g-o')
    

grid on 
daspect([1,1,1]) %For equal data unit lengths in all directions
xlabel('x in mm') %text in x-coordinate
ylabel('y in mm') %text in y-coordinate
zlabel('z in mm') %text in z-coordinate

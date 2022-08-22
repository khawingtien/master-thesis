a = [1,2,3];
rP = [];

b =  [ 17    17   -17   -17    17    17   -17   -17
    17   -17   -17    17    17   -17   -17    17
   300   300   300   300  -300  -300  -300  -300];


for quar_z = 1
    for quar_x = 1
%         for b_i = 1:8
%             b_index = (b(:,b_i))'

        quat = quaternion([quar_x, 0, quar_z],'eulerd','XYZ','point');
        rP_temp = rotatepoint(quat,[b(:,1)'; b(:,2)'; b(:,3)']);
        rP = [rP rP_temp] %append rP
%         end
    end 
end

%%Comparison 
 rotation = [1 0 0 deg2rad(-10)]; %rotation at x-axis
 R_x = axang2rotm(rotation);

 rotation_z = [0 0 1 deg2rad(0)]; %rotation at z-axis
 R_z = axang2rotm(rotation_z);

 b_rot = (R_x * b); %rotation matrix 





 A = [4 -2 1];
B = [1 -1 3];
C = cross(A,B)
D = cross(B,A)
dot(C,A)==0 & dot(C,B)==0


for dott = 1:8
    C = cross(b_rot(:,dott),u(:,dott));
  dot(C,b_rot(:,dott))==0 & dot(C,u(:,dott))==0
end
  
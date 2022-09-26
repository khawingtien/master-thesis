figure
colorstring = 'kbgr';
rotation_z_angle = [1 10 20 30];
for ix = 1:length(rotation_z_angle)
rotation_angles = rotation_z_angle(ix);
X = zeros(1,1);
Y = zeros(1,1);
Z = zeros(1,1);
U = rot_axis(rotation_angles,1)*100;
V = rot_axis(rotation_angles,2)*100;
W = rot_axis(rotation_angles,3)*100;
quiver3(X,Y,Z,U,V,W,'Color', colorstring(ix));
hold on 

%plot Rod 
Rod_top = [0 0 b(3,1)];
Rod_bottom = [0 0 b(3,5)];
Rod = [Rod_top; Rod_bottom]';
plot3(Rod(1,:), Rod(2,:), Rod(3,:),'LineWidth',2,'Color','k')
hold on 

    for i = 1:length(rotation_angles_3Daxis)
        rot_angle = rotation_angles_3Daxis(i);
        R = axang2rotm([rot_axis(rotation_angles,:), deg2rad(rot_angle)]);
        POI_vector = R *Rod_bottom';
        plot3(POI_vector(1,:),POI_vector(2,:),POI_vector(3,:),'o','Color', colorstring(ix))
        hold on 
    end


end
xlabel ('x-axis')
ylabel ('y-axis')
zlabel ('z-axis')

title ("Rotation angle for 3D workspace")
axis equal
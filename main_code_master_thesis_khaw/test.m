
r = 150; %in mm 
[X,Y,Z] = cylinder(r);
h = 200; %in mm
Z = (Z*h)-100;
figure
s = surf(X,Y,Z,'FaceColor','r','FaceAlpha','0.5')
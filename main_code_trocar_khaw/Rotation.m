s = surf(peaks(20));
%Trocar Point with rod length = 320mm
bx = 0.017; %in m
by = 0.017; %in m
bz = 0.46;  %in m
b30 = [bx  bx  -bx    -bx  bx   bx    -bx   -bx;
       by  -by  -by   by   by    -by    -by  by;
       0.12  0.12   0.12    0.12   -bz    -bz   -bz  -bz];
b30 = b30.*1000; %in mm

s = surf([b30(1,:),b30(2,:),b30(3,:)]);
xlabel('x-axis')
ylabel('y-axis')
zlabel('z-axis')

direction = [1 0 0];
rotate(s,direction,25)
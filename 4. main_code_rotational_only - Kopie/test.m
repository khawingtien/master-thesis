load('20220814_matlab.mat')
figure 
for i = 1:360
    for j = 1:30 
x =  position_bowl_360{1,i}{1,j}(1);
y =  position_bowl_360{1,i}{1,j}(2);
z =  position_bowl_360{1,i}{1,j}(3);
surf(x,y,z)
    end
end

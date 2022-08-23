log_array = [true, true, true, true, true, true, true, true];
A = [4 4 1 3 2 7 8 9]';
DoF = 4;
f_min = 5;
r = 8-DoF;

while r ~= 0
    if any (A< f_min)
    [fail_diff, f_id] = min(A-f_min);
    log_array (f_id) = 0 %%Challenge 
    A (f_id) = []
%     A = A* rand(8,1);
    r = length(A)-DoF;
    end 
end 

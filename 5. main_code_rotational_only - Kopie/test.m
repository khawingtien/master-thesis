log_array = [true, true, true, true, true, true, true, true];
A = [4 4 1 3 2 7 8 9]';
DoF = 4;
f_min = 5;
r = length(A)-DoF;
while r >= 0
    if any (A< f_min)
    [A_diff, f_id] = min(A-f_min);
    log_array (f_id) = 0 %%Problem here  
    A (f_id) = intmax; 
    r = r-1; %r will be reduced by one in every loop 
    end 
end 
%% Function berechnungSeilkraftverteilung 
function [stop,R,l] = berechnungSeilkraftverteilung_KHAW(r, a, b, f_min, f_max, rotation, w_p, w_p_t, rotation_w_p, pulley_kin, rad_pulley, R_A, rot_angle_A)
% Berechnung der improved closed-form Lösung aus "Cable-driven parallel robots, Pott"

% Basispunkte Roboter
 noC =8;

if any(b)  %if any element of b is nonzero = logical 1  (b is endeffector)
   % motion_pattern = 3; %1R2T
    motion_pattern = 5; %2R3T
else
    motion_pattern = 2; %2T %if element b is zeors, then motion pattern = 2
    disp('motion pattern failure');
end

r = repmat(r, 1, noC); %r for workspace position, in order to achieve the dimension (1,noC) 
R = axang2rotm(rotation); %axis angle to rotation matrix [0 0 1 angle] to Matrix Dimension=(2,2)
b_rot = R * b;

if pulley_kin == 'no'
    % Schließbedingung Vektoren 
    l = a - r - b_rot; 
elseif pulley_kin == 'yes'
    %calculate bx, by 
    %r_0_A = a;
    %R_A * b_A + a = r + b_rot;
    for i = 1 : noC
        b_A(:, i) = R_A(:, :, i) \ (r(:, i) + b_rot(:, i) - a(:, i));
    end 
    b_x_A = b_A(1, :);
    b_y_A = b_A(2, :);
    %calculate cable length 
    for i = 1 : noC
        l(i) = (acos((sqrt(b_x_A(1, i)^2 - 2*b_x_A(1, i)*rad_pulley + b_y_A(1, i)^2)) / (sqrt((b_x_A(1, i) - rad_pulley)^2 + b_y_A(1, i)^2))) + acos(b_y_A(1, i) / (sqrt((b_x_A(1, i) - rad_pulley)^2 + b_y_A(1, i)^2)))) * rad_pulley + sqrt(b_x_A(1, i)^2 - 2*b_x_A(1, i)*rad_pulley + b_y_A(1, i)^2);
    end
end 
    
%1st Check: für Arbeitsraum Berechnung: check ob length = 0 --> leads to NaN in u(unit vector)
for check_l = 1 : noC
    if l(:, check_l) == zeros(3,1)
        stop = 1; %violation exist 
        %     f_closedform = 0; %tbd
        %     f = 0; %tbd weil hier noch nicht definiert, daher kein output möglich
        return %Return control to invoking script or function            
    end
end

%Define Einheitsvektoren
u = zeros(3,noC);
if pulley_kin == 'no'
    for i=1:noC
        u(:,i) = l(:,i) / norm(l(:,i));
    end
elseif pulley_kin == 'yes'
    for i = 1 : noC
        u(:, i) = R_A(:, :, i) * [-sind(rot_angle_A(:, i)); cosd(rot_angle_A(:, i))]; %Cosine of argument in degrees
    end 
end 

b_cross_u = zeros(3,noC);
for i=1:noC
    b_cross_u(:,i) = cross(b(:,i),u(:,i)); %from Artur 3D vector
end

% Strukturmatrix
A_T = [u; b_cross_u];
A_T(~any(A_T,2),:) = []; %when all values in Dimension 2 (row) == 0, then delete the row.     Löscht die leeren Zeilen, z.B. fuer den 3R, 2T Roboter
                          %(any)Determine if any array elements are nonzero
                          
%2.Check if robot is in a nonsingular posn --> A_T full row rank
rank_A_T = size(orth(A_T.').', 1); %Orthonormal basis for range of matrix (Pott page 93)
    %nonsingular posn
if rank_A_T == size(A_T, 1)-1 %%%%For 8-wires_robot.py (need to MINUS 1) dunno why!
    %disp('non singular posn')
else
    %sigular posn
    %disp('singular posn')
    stop = 1; %violation exist 
    return
end
    
% Closed-form method
f_M = ones(noC(1),1);
f_M = f_M .* ((f_min + f_max) / 2);

A_inv = pinv(A_T); % Moore-Penrose Inverse

%wrench berechnen unter Berücksichtigung von Rotationen definiert in rotation_w_p
rotation_wrench_p = axang2rotm(rotation_w_p);
rotation_wrench_p =  repmat(rotation_wrench_p,1,2);
% rotation_wrench_p = rotation_wrench_p(1:size(A_inv, 2), 1:size(A_inv, 2)); %crop rotation matrix to 2 or 3-dimensional
wrench_p = zeros(size(A_inv, 2), 1);
wrench_p(1, 1) = w_p;
wrench_p = rotation_wrench_p * wrench_p;

%tbd momentan x, y oder torque wegen rot 
if size(A_inv, 2) == 3
    wrench_p(3, 1) = w_p_t;
end 

f_V = -A_inv * (wrench_p + A_T * f_M); %Gleichung 3.59 Pott Buch
if norm(f_V, 2) >= (1/2 * (f_max - f_min)) && norm(f_V, 2) <= (1/2 * sqrt(noC) * (f_max - f_min)) %norm(f_V,2) as p-norm of a vector =2, gives the vector magnitude or Euclidean length of the vector
    %disp("fail to provide a feasible solution although such a solution exists")
elseif norm(f_V, 2) > (1/2 * sqrt(noC) * (f_max - f_min))
    %disp("No solution exists") %if norm(f_V,2) violates the upper limit, no solution exist. 
    stop = 1; %violation exist
    return
end

f = f_M + f_V;
% f_closedform = f;

%Improved closed-form
% Pruefen, ob eine Kraft die Kraftgrenzen verletzt und die Kraft mit der
% groessten Differenz wählen
for counter_closed_form = 1 : (noC - motion_pattern)
    f_fail = 0;
    fail_diff = 0;
    for i = 1 : noC
        if f(i) < f_min   %%Case 1 if f(i) <5N, then set it to 0N. 
            if f_fail == 0 %Case 1a
                f_fail = f(i); %original value
                fail_diff = f_min - f_fail; %suppose to be 5N
            else
                if (f_min - f(i)) > fail_diff %Case 1b
                    f_fail = f(i); %original value
                    fail_diff = f_min - f_fail;
                end
            end
            
        elseif f(i) > f_max %%Case 2
            if f_fail == 0
                f_fail = f(i);
                fail_diff = f_fail - f_max;
            else
                if(f(i) - f_max) > fail_diff
                    f_fail = f(i);
                    fail_diff = f_fail - f_max;
                end
            end
        end
    end
    
    % Wenn eine Kraft die Kraftgrenzen verletzt
    if fail_diff ~= 0 %tbd Artur: f_fail
        %     f = round(f, 5);
        %     f_fail = round(f_fail, 5);
        index_f_fail = find(f == f_fail);
        
        A_T_neu = A_T;
        A_T_neu(:, index_f_fail) = [];
        A_inv_neu = pinv(A_T_neu);
        
        f_neu = f;
        f_neu(index_f_fail) = [];
        
        f_M_neu = f_M;
        f_M_neu(index_f_fail) = [];
        
        w_p_neu = f_min .* A_T(:, index_f_fail) + wrench_p;
        
        %tbd test
        f_neu = A_inv_neu * (- w_p_neu); %Lösung des Problems Af + w = 0
        
        counter_f_neu = 1;
        for j = 1 : noC
            if f(j) ~= f_fail
                f(j) = f_neu(counter_f_neu);
                counter_f_neu = counter_f_neu + 1;
            end
        end
        f(f == f_fail) = f_min; %Subsitute the logic when f==f_fail with 5N, so the f_min range can be fulfilled 
    end
end
%% check static equlibrium
%Force
for i = 1 : noC
    array_sum_force(:, i) = f(i) .* u(:, i);
end
sum_f = sum(array_sum_force, 2); %sum(A,dimension 2)is a column vector containing the sum of each row.
sum_f = sum_f + wrench_p(1:3);
sum_f = round(sum_f, 5); %round to 5 digits
stop = 0;
if any(sum_f, 'all') %Determine if any array elements are nonzero, test over ALL elements of sum_f with the command 'all'
    stop = 1;
    return
end
%torque
if size(A_T, 1) == 3 %TOASK: Only for degree of freedom = 3???
    for i = 1 : noC
        %     array_sum_torque(:,i) = array_sum_force (1, i) * b_rot(2, i) + array_sum_force(2, i) * b(1, i);
        array_sum_torque(:,i) = f(i) * A_T(3, i);
    end
    sum_torque = sum(array_sum_torque, 2);
    sum_torque = sum_torque + wrench_p(3);
    sum_torque = round(sum_torque, 5);
else
    sum_torque = 0;
end
stop = 0;
if any(sum_torque, 'all') %Determine if any array elements are nonzero, test over ALL elements of sum_torque with the command 'all'
    stop = 1;
    return
end
%% display info 
if find(f > f_max)
%        disp("Achtung! Seilkraft ueberschreitet den Maximalwert");
    stop = 1;
elseif find(f < f_min)
%        disp("Achtung! Seilkraft unterschreitet den Minimalwert")
    stop = 1;
end
end
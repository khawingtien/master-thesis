%% Function berechnungSeilkraftverteilung 
function [stop] = berechnungSeilkraftverteilung_KHAW(ws_position, a, b, f_min, f_max,noC, b_rot,  POI_rot, w_p_x, w_p_t,  rotation_matrix, pulley_kin, rad_pulley, R_A, limit, f_direction,~)

%Define Point of Interest (POI) 
% POI_offset = [0 0 b(3,5)]'; %first value of endeffector in z-axis (offset to half the rod length)

%rotate the POI in accordance with the rotation angle 
% R = axang2rotm(rotation); %axis angle to rotation matrix [0 1 0 angle] to Matrix Dimension=(3,3)
% POI_rot = R_x * POI_offset; %the position of the POI after rotation at (0,0,0)

ws_position = ws_position - POI_rot; % reverse calculation to trocar point (ORIGIN)

% Basispunkte Roboter
ws_position = repmat(ws_position, 1, noC); %ws_position for workspace position, in order to achieve the dimension (1,noC) 

% R = axang2rotm(rotation); %axis angle to rotation matrix [0 0 1 angle] to Matrix Dimension=(3,3)
% b_rot = R * b; [b_rot as input arguement already) 


    if pulley_kin == 'no'
        % Schließbedingung Vektoren (Closure constrain v_i) Equation 3.1 & 3.2 in Pott's Book 
        l = a - ws_position - b_rot; 
    elseif pulley_kin == 'yes'
        %calculate bx, by 
        %r_0_A = a;
        %R_A * b_A + a = ws_position + b_rot;
        for i = 1 : noC
            b_A(:, i) = R_A(:, :, i) \ (ws_position(:, i) + b_rot(:, i) - a(:, i));
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
            return %Return control to invoking script or function            
        end
    end

%Define Einheitsvektoren (Unit vector) 
u = zeros(3,noC);
for i=1:noC
    u(:,i) = l(:,i) / norm(l(:,i)); %Equation 3.3 Pott's book 
%     u2(:,i) = l(:,i) / norm(l(:,i),2); norm with p = 2 (no different to
%     p=1 )
end

%Calculate cross product 
b_cross_u = zeros(3,noC);
for i=1:noC
    b_cross_u(:,i) = cross(b_rot(:,i),u(:,i));  
end

%Suppose to be true? to check if the C is perpendicular to the other two
%(20220822)
%Perpendicular Check 
for dott = 1:8
    C = cross(b_rot(:,dott),u(:,dott));
  dot(C,b_rot(:,dott))==0 & dot(C,u(:,dott))==0;
end
  

% Strukturmatrix
A_T = [u; b_cross_u];
                           
% 2.Check if robot is in a nonsingular posn --> A_T full row rank
rank_A_T = size(orth(A_T.').', 1); %Orthonormal basis for range of matrix (Pott page 93)
    %nonsingular posn
if rank_A_T == size(A_T, 1) %%For 8-wires_robot.py with 0° Rotation (need to MINUS 1) dunno why!1
    %disp('non singular posn')
else
    %sigular posn
    %disp('singular posn')
    stop = 1; %violation exist 
    return
end
    
% Closed-form method
f_M = ones(noC(1),1); 
f_M = f_M .* ((f_min + f_max) / 2); % f_M = average feasible force (below Eq 3.53 Pott Book)
A_inv = pinv(A_T); % Moore-Penrose Inverse

%Implementation of wrench either in x-axis or y-axis 
[wrench_p_f, ~] = wrench_khaw2(b_rot,w_p_x,w_p_t,rotation_matrix,f_direction);

f_V = -A_inv * (wrench_p_f + A_T * f_M); %Gleichung 3.55 & 3.59 Pott Buch
%  einheit_matrix  = A_T*A_inv;

% w_v = (A_T*A_T')\(-wrench_p_f - A_T*f_M);
% f_V = A_T'*w_v;

norm_f_V = norm(f_V, 2)
if norm_f_V >= limit.lower && norm(f_V, 2) <= limit.upper %norm(f_V,2) as p-norm of a vector =2, gives the vector magnitude or Euclidean length of the vector Equation 3.6 Pott's book 
    disp("fail to provide a feasible solution although such a solution exists")
elseif norm_f_V > limit.upper
   %disp("No solution exists") %if norm(f_V,2) violates the upper limit, no solution exist. 
    % if it below the lower limit, the force distribution is feasible
% elseif norm_f_V 
% 
    Kappa = zeros(1,3);

    k = null(A_T);
        for i = 1:size(k,2)
            k_col=k(:,i);
            if  min(k_col) > 0
                Kappa(i) = min(k_col)/max(k_col);
                
            elseif max(k_col) < 0
                Kappa(i) = max(k_col)/min(k_col);
                
            else
                Kappa(i) = 0;
            end 
        end
            if any(Kappa)
                stop = 0;
            else
                stop = 1;
            end


    stop = 1; %violation exist
     return
end

f = f_M + f_V; %Eq 3.53 Pott Book (feasible force + arbitrary force vector)

%Improved closed-form
% Pruefen, ob eine Kraft die Kraftgrenzen verletzt und die Kraft mit der
% groessten Differenz wählen
DOF = 6; %3R3T
r = noC - DOF; %r = m-n
f_id_mat = [];
A_T_neu = A_T;
f_neu = f;
no_reduction = false;
f_id = noC+1;
log_array = [true, true, true, true, true, true, true, true];

%Compute the solution by recursively reducing the order until the
%degree-of-redundancy become r= 0 (Pott book pg 95)
while r ~= 0 %calculate redundancy
% for counter_closed_form = 1 : (noC - DOF)
%     f_fail = 0;
%     fail_diff = 0;
    f_id_old=f_id;
    if any(f_neu<f_min) %condition check if cable force violate the minimum force
        [fail_diff, f_id] = min(f_neu-f_min); %find the maximum difference of the cable force (use min as command as its ans is negative)
%         f_fail=f_neu(f_id);
    elseif any(f_neu>f_max)
        [fail_diff, f_id] = max(f_neu+f_max);
%         f_fail=f_neu(f_id);
    else
        no_reduction=true;
        break
    end

    % Wenn eine Kraft die Kraftgrenzen verletzt
    if fail_diff ~= 0 %tbd Artur: f_fail (static equilibrium is not zero)
               
        A_T_neu(:, f_id) = []; %remove the index that violate force limit
         
        A_inv_neu = pinv(A_T_neu);
        w_p_neu = f_min * A_T(:, f_id) + wrench_p_f; %Equation 3.61 Pott's book  TO CHECK (WARUM NUR EINE SPALTEN)? 22.08.2022
        f_neu = A_inv_neu * (- w_p_neu); %Lösung des Problems Af + w = 0 nach f_neu
        r = length(f_neu) - DOF; %r = m-n

        if f_id >= f_id_old
            offset=1;
        else
            offset=0;
        end
        f_id_mat = [f_id_mat;f_id+offset]; %Subsitute the logic when f==f_fail with 5N, so the f_min range can be fulfilled 
    end 
end

if no_reduction == false
    log_array(f_id_mat) = 0;
%     f(f_id_mat) = f_min; %Subsitute the logic when f==f_fail with 5N, so the f_min range can be fulfilled 
    A_T (:,log_array) = A_T_neu; %true situation
    A_T (:,~log_array) = zeros(6,2-r); %false situation, subsitute with zeros
    f(log_array) = f_neu;
    f(~log_array) = f_min;

    wrench_p_f = w_p_neu;
%     f(f~=f_min) = f_neu; %Subsitute the f_neu into the f
end

%% check static equlibrium
%Force
sum_f = A_T(1:3,:)* f;
sum_f = sum_f + wrench_p_f(1:3,:); %% f + [f_x f_y f_z]  Equation 3.5 Pott's book 
sum_f = round(sum_f, 0); %round to 5 digits %%WARNING TODO
stop = 0; %static equilibrium fulfill

if any(sum_f, 'all') %Determine if any array elements are nonzero, test over ALL elements of sum_f with the command 'all'
    stop = 1; %static equilibrium not fulfill
    return
end
%torque
sum_torque = A_T(4:6,:) * f;
sum_torque = sum_torque + wrench_p_f(4:6); 
sum_torque = round(sum_torque, 0); %%WARNING TODO
stop = 0; %static equilibrium fulfill, no violation of force distribution 

if any(sum_torque, 'all') %Determine if any array elements are nonzero, test over ALL elements of sum_torque with the command 'all'
    stop = 1; %the static equilibrium was not fulfilled 
    return
end

%% display info 
if find(f > f_max)
%        disp("Achtung! Seilkraft ueberschreitet den Maximalwert");
    stop = 1;
    return


elseif find(f < f_min)
       disp("Achtung! Seilkraft unterschreitet den Minimalwert")
  stop = 1;

end
end



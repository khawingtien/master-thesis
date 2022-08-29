%% Function berechnungSeilkraftverteilung 
function [stop] = berechnungSeilkraftverteilung_KHAW(ws_position, a, b, f_min, f_max, noC, b_rot_xy, w_p_x, w_p_t,  rotation_matrix,  limit, f_direction,POI_rot)
% Berechnung der improved closed-form Lösung aus "Cable-driven parallel robots, Pott"
% Basispunkte Roboter
ws_position = repmat(ws_position, 1, noC); %ws_position for workspace position, in order to achieve the dimension (1,noC) 

% Schließbedingung Vektoren (Closure constrain v_i) Equation 3.1 & 3.2 in Pott's Book 
%     l = a - ws_position - b_rot; 
    l = a - ws_position - b_rot_xy;

%1st Check: für Arbeitsraum Berechnung: check ob length = 0 --> leads to NaN in u(unit vector)
for check_l = 1 : noC
    if l(:, check_l) == zeros(3,1)
        stop = 1; %violation exist 
        %     f_closedform = 0; %tbd
        %     f = 0; %tbd weil hier noch nicht definiert, daher kein output möglich
        return %Return control to invoking script or function            
    end
end

%Define Einheitsvektoren (Unit vector) 
u = zeros(3,noC);
for i=1:noC
    u(:,i) = l(:,i) / norm(l(:,i)); %Equation 3.3 Pott's book 
end


b_cross_u = zeros(3,noC);
for i=1:noC
    b_cross_u(:,i) = cross(b_rot_xy(:,i),u(:,i)); 
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
f_M = ones(noC,1); %preallocating for speed
f_M = f_M .* ((f_min + f_max) / 2); % f_M = average feasible force (below Eq 3.53 Pott Book)
A_inv = pinv(A_T); % Moore-Penrose Inverse

%Implementation of wrench either in x-axis or y-axis 
[wrench_p_f, ~] = wrench_khaw2(b,w_p_x,w_p_t,rotation_matrix,f_direction,POI_rot);

f_V = -A_inv * (wrench_p_f + A_T * f_M); %Gleichung 3.55 & 3.59 Pott Buch
norm_f_V = norm(f_V, 2);
if norm_f_V  >= limit.lower && norm(f_V, 2) <= limit.upper %norm(f_V,2) as p-norm of a vector =2, gives the vector magnitude or Euclidean length of the vector Equation 3.6 Pott's book 
%    disp("fail to provide a feasible solution although such a solution exists")
elseif norm_f_V > limit.upper
   %disp("No solution exists") %if norm(f_V,2) violates the upper limit, no solution exist. 
    % if it below the lower limit, the force distribution is feasible
%     stop = 1;
            Kappa = zeros(1,3);
            k = null(A_T); %nullspace of A_T (one-Dimensional Kernel) so that A_T*k = 0 (Pott pg167) eq 5.7
            
            for i = 1:size(k,2)
                k_col=k(:,i);
                if  min(k_col) > 0 %eq 5.8
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
    return
end

f = f_M + f_V; %Eq 3.53 Pott Book (feasible force + arbitrary force vector)

%Improved closed-form solution (update on 24.08.2022)
% Pruefen, ob eine Kraft die Kraftgrenzen verletzt und die Kraft mit der
% groessten Differenz wählen
DOF = 3; %2R1T
r = noC - DOF; %r = m-n
log_array = ~zeros(1,8);

%Compute the solution by recursively reducing the order until the
%degree-of-redundancy become r= 0 (Pott book pg 95)
while r ~= 0 %calculate redundancy

    if any(f(log_array)<f_min) %condition check if cable force violate the minimum force
        [fail_diff, f_id] = min(f-f_min); %find the maximum difference of the cable force (use min as command as its ans is negative)
    elseif any(f(log_array)>f_max)
        [fail_diff, f_id] = max(f+f_max);
    else 
        break
    end

    % Wenn eine Kraft die Kraftgrenzen verletzt
        log_array(f_id) = false;
        A_inv_neu = pinv(A_T(:,log_array));
        w_p_neu = f_min * A_T(:, f_id) + wrench_p_f; %Equation 3.61 Pott's book  TO CHECK (WARUM NUR EINE SPALTEN)? 22.08.2022
        f(log_array) = A_inv_neu * (- w_p_neu); %Lösung des Problems Af + w = 0 nach f_neu
        r = length(f(log_array)) - DOF; %r = m-n

%Update the new values
    A_T (:,f_id) = zeros(6,1); %false situation, subsitute with zeros

    if fail_diff <0
        f(f_id) = f_min; 
    else 
        f(f_id) = f_max;
        wrench_p_f = w_p_neu;
    end
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
%        disp("Achtung! Seilkraft unterschreitet den Minimalwert")
  stop = 1;

%wrench-closure workspace 
% Kappa = zeros(1,3);
% k = null(A_T);
% for i = 1:3
%     k_col=k(:,i);
%     if  min(k_col) > 0
%         Kappa(i) = min(k_col)/max(k_col);
%         
%     elseif max(k_col) < 0
%         Kappa(i) = max(k_col)/min(k_col);
%         
%     else
%         Kappa(i) = 0;
%     end 
% end
% if any(Kappa)
%     stop = 0;
% else
%     stop = 1;
% end
% 

end
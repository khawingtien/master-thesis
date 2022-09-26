%% Function berechnungSeilkraftverteilung 
function [stop, cable_length] = berechnungSeilkraftverteilung_KHAW(ws_position, a, f_min, f_max, noC, b_rot_xy, wrench,limit,f_M)
% Berechnung der improved closed-form Lösung aus "Cable-driven parallel robots, Pott"

% Basispunkte Roboter
ws_position = repmat(ws_position, 1, noC); %ws_position for workspace position, in order to achieve the dimension (1,noC) 

% Schließbedingung Vektoren (Closure constrain v_i) Equation 3.1 & 3.2 in Pott's Book 
    l = a - ws_position - b_rot_xy;

%1st Check: für Arbeitsraum Berechnung: check ob length = 0 --> leads to NaN in u(unit vector)
for check_l = 1 : noC
    if l(:, check_l) == zeros(3,1)
        stop = 1; %violation exist 
        return %Return control to invoking script or function            
    end
end

%Define Einheitsvektoren (Unit vector) 
cable_length = zeros(1,8); 
u = zeros(3,noC);
for i=1:noC
    u(:,i) = l(:,i) / norm(l(:,i)); %Equation 3.3 Pott's book 
    cable_length(1,i) = norm(l(:,i));
end

b_cross_u = zeros(3,noC);
for i=1:noC
    b_cross_u(:,i) = cross(b_rot_xy(:,i),u(:,i)); %%b_rot_xy
end

DoF = 5; %2R3T 
% Strukturmatrix
A_T = [u; b_cross_u]; %Jacobian matrix
A_T = A_T (1:DoF,:); %only 5DoF  
wrench = wrench(1:DoF,:); %only 5DoF  

% 2.Check if robot is in a nonsingular posn --> A_T full row rank
% rank_A_T = size(orth(A_T.').', 1); %Orthonormal basis for range of matrix (Pott page 93)
rank_A_T = rank(A_T);


if rank_A_T >= DoF %2R3T 
%     disp('non singular posn')
else
%     rank_A_T < DoF 
    %disp('singular posn')
    stop = 1; %violation exist 
    return
end
    
%% Closed-form method
% A_inv = pinv(A_T); % Moore-Penrose Inverse
% f_V = -A_inv * (wrench + A_T * f_M); %Gleichung 3.55 & 3.59 Pott Buch

w_v = (A_T*A_T')\(-wrench - A_T*f_M); %Alternative from Pott Email
f_V = A_T'*w_v; %Alternative from Pott Email (same answer as pinv) 

norm_f_V = norm(f_V, 2);
if norm_f_V  >= limit.lower && norm(f_V, 2) <= limit.upper %norm(f_V,2) as p-norm of a vector =2, gives the vector magnitude or Euclidean length of the vector Equation 3.6 Pott's book 
%    disp("fail to provide a feasible solution although such a solution exists")
elseif norm_f_V > limit.upper
   %disp("No solution exists") %if norm(f_V,2) violates the upper limit, no solution exist. 
    % if it below the lower limit, the force distribution is feasible
%      stop = 1;
%      return 
        %% wrench-closure workspace
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
                return
            end
end

f = f_M + f_V; %Eq 3.53 Pott Book (feasible force + arbitrary force vector)

%Improved closed-form solution (update on 04.09.2022)
% Check whether a force violates the force limits and select the force with the largest difference.
r = noC - DoF; %r = m-n
log_array = ~zeros(1,noC);

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
        A_inv_neu = pinv(A_T(:,log_array)); %A_inv_neu with a colum dropped. 
        w_p_neu = f_min * A_T(:,f_id) + wrench; %Equation 3.61 Pott's book, (AT_i denotes the i-th column of the matrix A_T) 
        f(log_array) = A_inv_neu * (-w_p_neu); %Lösung des Problems Af + w = 0 nach f_neu
        r = length(f(log_array)) - DoF; %r = m-n

%Update the new values
    A_T (:,f_id) = zeros(DoF,1); %false situation, subsitute with zeros

    if fail_diff <0
        f(f_id) = f_min; 
    else 
        f(f_id) = f_max;
        wrench = w_p_neu;
    end
end

%% check static equlibrium
%Force
sum_f = A_T(1:3,:)*f;
sum_f = sum_f + wrench(1:3,:); %% f + [f_x f_y f_z]  Equation 3.5 Pott's book 
sum_f = round(sum_f, 0); %round to 5 digits %%WARNING TODO
% stop = 0; %static equilibrium fulfill

if any(sum_f, 'all') %Determine if any array elements are nonzero, test over ALL elements of sum_f with the command 'all'
    stop = 1; %static equilibrium not fulfill
    return
end

%torque
sum_torque = A_T(4:5,:)*f;
sum_torque = sum_torque + wrench(4:end); 
sum_torque = round(sum_torque, 10);
stop = 0; %static equilibrium fulfill, no violation of force distribution 

if any(sum_torque, 'all') %Determine if any array elements are nonzero, test over ALL elements of sum_torque with the command 'all'
    stop = 1; %the static equilibrium was not fulfilled 
    return
end

%% display info 
if find(f > f_max)
%       disp("Achtung! Seilkraft ueberschreitet den Maximalwert");
    stop = 1;
    return

elseif find(f < f_min)
%      disp("Achtung! Seilkraft unterschreitet den Minimalwert")
  stop = 1;

end
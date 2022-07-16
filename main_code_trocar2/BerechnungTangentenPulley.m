function [Lfill, Rfill, tang_pulley_outer, R_A, rot_angle_A] = BerechnungTangentenPulley(loc_winch, rad_winch, rad_pulley, MP_pulley)
figure()
count_L = 1;
count_R = 1;
for i = 1 : size(MP_pulley, 2)
    x_diff = MP_pulley(1, i) - loc_winch(1, i);
    y_diff = MP_pulley(2, i) - loc_winch(2, i);
    d2 = x_diff ^ 2 + y_diff ^ 2;
    r21 = (rad_pulley - rad_winch) / d2;
    s21 = sqrt(d2 - (rad_pulley - rad_winch)^2) / d2; % <-- If d2<(r2-r1)^2, no solution is possible
    u1 = [-x_diff * r21 - y_diff * s21, -y_diff * r21 + x_diff * s21]; % Left unit vector direction
    u2 = [-x_diff * r21 + y_diff * s21, -y_diff * r21 - x_diff * s21]; % Right unit vector
    
    L1 = [loc_winch(1, i), loc_winch(2, i)] + rad_winch * u1;
    L2 = [MP_pulley(1, i), MP_pulley(2, i)] + rad_pulley * u1; % Left line tangency points
    L = [L1; L2];
    Lfill(1, count_L) = L1(1);
    Lfill(2, count_L) = L1(2);
    count_L = count_L + 1;
    Lfill(1, count_L) = L2(1);
    Lfill(2, count_L) = L2(2);
    count_L = count_L + 1;
    R1 = [loc_winch(1, i), loc_winch(2, i)] + rad_winch * u2;
    R2 = [MP_pulley(1, i), MP_pulley(2, i)] + rad_pulley * u2; % Right line tangency points
    R = [R1; R2];
    Rfill(1, count_R) = R1(1);
    Rfill(2, count_R) = R1(2);
    count_R = count_R + 1;
    Rfill(1, count_R) = R2(1);
    Rfill(2, count_R) = R2(2);
    count_R = count_R + 1;
    
    
    %% plot
    %     hold on
    %     plot(L(:, 1), L(:, 2), 'x:b');
    %     hold on
    %     plot(R(:, 1), R(:, 2), 'x:r');
    %     hold on
    %     viscircles(transpose(loc_winch(:, i)), rad_winch, 'Color', 'k');
    %     hold on
    %     viscircles(transpose(MP_pulley(:, i)), rad_pulley, 'Color', 'k');
    %     hold on
    %     %     plot(a(1, :), a(2, :), '.k');
    %     %     hold on
    %     %     a_adapt = a;
    %     %     a_adapt(1,5) = a(1,1);
    %     %     a_adapt(2,5) = a(2,1);
    %     %     % a_adapt(3,5) = 0;
    %     %     plot(a_adapt(1, :), a_adapt(2, :), 'k--');
    %     axis([-450 450 -450 450])
    %     axis square
    %     hold on
end

tang_pulley_outer = zeros(2, 4);
tang_pulley_outer = [Rfill(:, 2), Lfill(:, 4), Rfill(:, 6), Lfill(:, 8)];

%Agerade entspricht Tangentialem Punkt an pulley, wenn Seil senkrecht von
%unten/oben kommen würde
Agerade (:, 1:2) = MP_pulley(:, 1:2) - [rad_pulley; 0]; % Seilrollen links
Agerade (:, 3:4) = MP_pulley(:, 3:4) + [rad_pulley; 0]; % Seilrollen rechts
for i = 1 : 4
    tangent_Agerade(i, :) = Agerade(:, i) - tang_pulley_outer(:, i);
    dist_tangent_Agerade(i) = norm(tangent_Agerade(i, :));
end
rot_angle_A = 2 .* asind(dist_tangent_Agerade ./ (2 .* rad_pulley));
rot_angle_A(1, [2, 4])  = - rot_angle_A(1, [2, 4]);
%create rotation matrix
R_A = zeros(3, 3, 4);
for i = 1 : 4
    rotation_A(i, :) = [0 0 1 ((pi/180) * rot_angle_A(:, i))];
    R_A(:, :, i) = axang2rotm(rotation_A(i, :));
end
R_A(3, :, :) = [];
R_A(:, 3, :) = [];
% nichts danach
end

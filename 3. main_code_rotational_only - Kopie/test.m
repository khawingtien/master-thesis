[ws_bowl_mat] = ws_position_bowl();

for bowl_index = 1: length(ws_bowl_mat)
    ws_position = ws_bowl_mat(:,bowl_index);
end

ax = 0.230; %in m
ay = 0.230; %in m 
az = 0.102; %in m

[a] = SetupParameter(ax,ay,az);
b_cell = endeffektor2();

f_min = 5;
f_max = 36; % fmax berechnet: 2* 183 / 10 = 36, 6 %Motor 
limit.lower = (1/2 * (f_max - f_min)) ; %upper limit for improve closed-form solution (eq. 3.6 Pott book)
limit.upper = (1/2 * sqrt(noC) * (f_max - f_min)); %lower limit for improved closed form (eq. 3.6 Pott book)
counter_analysis = 1; %tbd counter logik ändern!!!!

[workspace_logical, R] = Arbeitsraum_khaw(a, b, f_min, f_max,noC, rotation, w_p_x, w_p_t, rotation_w_p, workspace_logical, pulley_kin, rad_pulley, R_A, rot_angle_A, coordinate, limit, f_direction);

[stop,R] = berechnungSeilkraftverteilung_KHAW(ws_position, a, b, f_min, f_max,noC, R, w_p_x, w_p_t,  rotation_matrix,  pulley_kin, rad_pulley, R_A, rot_angle_A, limit, f_direction,POI_rot);
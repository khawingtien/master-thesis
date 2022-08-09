tic
%counter
counter = 1;

%parameter 
ax = 0.230; %in m
ay = 0.230; %in m 
az = 0.102; %in m

%calculation of frame 
[a] = SetupParameter(ax,ay,az);

%Endeffector matrix
b = endeffektor2();
b =  b{1,1};

%Parameter definition
f_min = 5;
f_max = 36; % fmax berechnet: 2* 183 / 10 = 36, 6 %Motor 
noC = length(a); 
limit.lower = (1/2 * (f_max - f_min)) ; %upper limit for improve closed-form solution (eq. 3.6 Pott book)
limit.upper = (1/2 * sqrt(noC) * (f_max - f_min)); %lower limit for improved closed form (eq. 3.6 Pott book)
f_directions = ["x","y"]; %define the f_x and f_y wrench direction. 

%Pulley definition
pulley_kin = 'no';
rad_pulley = 9.37+0.5;
R_A = 1; %just for input, is not in use 

%wrench definition
w_p_x = 0;
w_p_t = 0;
grid_deg = 8;

% %calculate rotation matrix for wrench 
if w_p_x == 0
    rotation_matrix.wpx = axang2rotm([1 0 0 0]); %Euler Winkel (x,y,z, Winkel), a rotation of 0 radians around the y-axis
    rotation_matrix.wpy = axang2rotm([0 1 0 0]);
else
discrete_rot_angle_w_p = linspace(0, 2*pi*(1-1/grid_deg), grid_deg)'; %(x1, 1/8 from the complete 360°, n) n Punkte zwischen x1 und x2 
    for i = 1 : size(discrete_rot_angle_w_p, 1) %K:calculate each of the rotation
    rotation_matrix.wpx = axang2rotm([1 0 0 0 (discrete_rot_angle_w_p)]); %rotation matrix for f_Y around X-AXIS (IMPORTANT)
    rotation_matrix.wpy = axang2rotm([0 1 0 0 (discrete_rot_angle_w_p)]); %rotation matrix for f_X around Y-AXIS (IMPORTANT)
    end 
end


[ws_bowl_mat, R_x_mat, R_z, R_x] = ws_position_bowl();

for bowl_index = 1: length(ws_bowl_mat)
    ws_position = ws_bowl_mat(:,bowl_index);
    R = R_x;

%Workspace calculation
    for f_xy=1:2 
    f_direction = f_directions(f_xy);
    ws_logical_bowl = zeros(1,size(ws_bowl_mat,2));
    [stop] = berechnungSeilkraftverteilung_KHAW(ws_position, a, b, f_min, f_max,noC, R, w_p_x, w_p_t,  rotation_matrix,  pulley_kin, rad_pulley, R_A, limit, f_direction);
    counter = counter + 1; %no semicolon, to show the current progression during debugging
       if stop == 0  %no violation of f_min & f_max (fulfil the requirements)(TRUE)
          ws_logical_bowl(1,bowl_index) = 1; %write 1 as TRUE
          elseif stop == 1 %if violation exist 
          ws_logical_bowl(1,bowl_index) = 0; %write 0 as FALSE 
       end
 
    
    %finalen Arbeitsraum bestimmen und darstellen
     counter = counter +   1
%     [analysis, workspace_logical, workspace_adapt_pointwise] = Arbeitsraum_Verarbeitung_KHAW(a, b, grid_n, b_name,  w_p_x, w_p_t, f_g, counter_analysis ,rot_name, analysis, coordinate, workspace_logical, R, noC);
    end
end 
toc
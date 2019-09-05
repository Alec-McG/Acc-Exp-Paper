function [vicon_alpha] = VICON_alpha(vicon_data,geo_know, x_axis_v,vicon_point, vicon_omega) 


r_p = geo_know(1);
b = geo_know(2);
l = geo_know(3);

cylinder_length = geo_know(4);

 
rx_r = vicon_data(1);
ry_r = vicon_data(2);
rz_r = vicon_data(3);
rw_r = vicon_data(4);
tx_r = (vicon_data(5));
ty_r = (vicon_data(6));
tz_r = (vicon_data(7));

rx_p = vicon_data(8);
ry_p = vicon_data(9);
rz_p = vicon_data(10);
rw_p = vicon_data(11);
tx_p = (vicon_data(12));
ty_p = (vicon_data(13));
tz_p = (vicon_data(14));


    %% Plot VICON robot
    
    
    vicon_omega = -vicon_omega+pi/2;

    % Get position of the robot  
    % Get Robot Points
    
geo_know(4) = vicon_point(2); %Set the Y coordinate
    
[wheel_1, wheel_2, castor] = get_wheels_sph(0.001, vicon_omega, geo_know );
    %% Calcs for forward model

    
hold on

% scatter3(wheel_1(1),wheel_1(2),wheel_1(3),'k')
% scatter3(wheel_2(1),wheel_2(2),wheel_2(3),'k')
% scatter3(castor(1),castor(2),castor(3),'k')
    

robot_0 = [wheel_1; wheel_2; castor]; 


% W = [    0     -0    1
%          0      0   -0
%         -1      0    0];
%     
% 
%         centre = (((wheel_1+ wheel_2)/2) + castor)/2;
%   
%     % Rotate by Omega
%     
%    
%     %w_first = asin(centre(3)/r_p);
%     w_first = atan(centre(1)/centre(3));
%     
%     %w_first = -atan(centre(3)/centre(1));
%     
%     R_w_first = eye(3) + sin(w_first).*W +(1-cos(w_first)).*W^2;
%     
%     
%     
% 
% 
% 
% robot(1,:) =R_w_first*robot(1,:)';     
% robot(2,:) =R_w_first*robot(2,:)';
% robot(3,:) =R_w_first*robot(3,:)';
% 
% 
% % scatter3(robot(1,1),robot(1,2),robot(1,3),'r')
% % scatter3(robot(2,1),robot(2,2),robot(2,3),'r')
% % scatter3(robot(3,1),robot(3,2),robot(3,3),'r')
%   
%     
% %rot_ax = [0 1 0];
% 
% R = eye(3) + sin(-v_omega).*W +(1-cos(-v_omega)).*W^2;
% 
% robot(1,:) =R*robot(1,:)';     
% robot(2,:) =R*robot(2,:)';
% robot(3,:) =R*robot(3,:)';
% 
% wheel_1 = robot(1,:);
% wheel_2 = robot(2,:);
% castor = robot(3,:);
% 
% centre_a = (((wheel_1+ wheel_2)/2) + castor)/2;
% 
% cent_shift = (centre_a - centre);
% 
% wheel_1(2) = wheel_1(2) - cent_shift(2);
% wheel_2(2) = wheel_2(2) - cent_shift(2);
% castor(2) = castor(2) - cent_shift(2);
% 
% robot(1,:) = wheel_1;
% robot(2,:) = wheel_2;
% robot(3,:) = castor;


%     scatter3(robot(1,1),robot(1,2),robot(1,3),'k')
%     scatter3(robot(2,1),robot(2,2),robot(2,3),'k')
%     scatter3(robot(3,1),robot(3,2),robot(3,3),'k')


% omega = 0 assumes drive wheel centre is point of measurement, need to
% change the point of measurement to centre of the robot (Done below)

% shift the points





hold on

% plot alpha = 0

[T, acc_x_opt, acc_y_opt, acc_z_opt, robot_angle, x_axis_r, midpoint] = plot_robot_3d_alpha(robot_0,'g',0);

% Plot with alpha


%% Move everything to origin
%robot
x_axis_r(1,:) = x_axis_r(1,:)-midpoint;
x_axis_r(2,:) = x_axis_r(2,:)-midpoint;
x_axis_r_vec = x_axis_r(1,:)'-x_axis_r(2,:)';

%scatter3(midpoint(1), midpoint(2), midpoint(3),10,'b','filled')

%vicon
x_axis_v(1,:) = x_axis_v(1,:) - vicon_point; 
x_axis_v(2,:) = x_axis_v(2,:) - vicon_point; 
x_axis_v_vec = x_axis_v(1,:)' - x_axis_v(2,:)';

%get rid of z components (?)

%x_axis_r_vec(3) = 0;
%x_axis_v_vec(3) = 0;

%angle between robot @ alpha = 0 and vicon is taken as alpha

if x_axis_v_vec(2) >0

    vicon_alpha = -atan2(norm(cross(x_axis_r_vec,x_axis_v_vec)),dot(x_axis_r_vec,x_axis_v_vec)); 

else 

     vicon_alpha = atan2(norm(cross(x_axis_r_vec,x_axis_v_vec)),dot(x_axis_r_vec,x_axis_v_vec)); 

end

end
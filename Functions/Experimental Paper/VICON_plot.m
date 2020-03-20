%% Function for VICON Alpha and Omega using Quaternions

function [accel_sim_vicon, x_axis, y_axis, z_axis, vicon_omega, vicon_alpha, vicon_point, side] = VICON_plot(vicon_data,geo_know)

fmt = {'Fontsize' 12, 'Fontweight', 'b'};


% Correcting Pipe Centre
% Values taken by fitting circle to points 

r_p = geo_know(1);
b = geo_know(2);
l = geo_know(3);


cylinder_length = geo_know(4);

x_c = geo_know(5);
z_c =  geo_know(6);
r_c_cor = geo_know(11);


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

quat_r = [rw_r, rx_r, ry_r, rz_r];
quat_p = [rw_p, rx_p, ry_p, rz_p];

%% Plot Real Pipe and Robot from Raw VICON Data

x = [250 0 0];
y = [0 -250 0];
z = [0 0 -250];

quat_p_inv = quatinv(quat_p);
quat_r_inv = quatinv(quat_r);
pipe_c_1 = quatrotate(quat_p_inv,[ 0 0  0]);
pipe_c_2 = quatrotate(quat_p_inv,[ 0 1000  0]);

vicon_point_original = [tx_r-x_c ty_r tz_r-z_c];

x_ax_rob_o_quat = quatrotate(quat_r_inv,x);
y_ax_rob_o_quat = quatrotate(quat_r_inv,y);
z_ax_rob_o_quat = quatrotate(quat_r_inv,z);


% Original Vicon Position and Pipe
% figure 
% hold on
% title('VICON Original')
% h = Cylinder( pipe_c_1, pipe_c_2,253.5,40,'black',0,1,0.1,1);
% 
% scatter3(vicon_point_original(1), vicon_point_original(2),vicon_point_original(3),30,'k','filled')
% 
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% axis equal
% 
% hold on
% 
% quiver3(vicon_point_original(1),vicon_point_original(2),vicon_point_original(3),x_ax_rob_o_quat(1),x_ax_rob_o_quat(2),x_ax_rob_o_quat(3),'color','r')
% quiver3(vicon_point_original(1),vicon_point_original(2),vicon_point_original(3),y_ax_rob_o_quat(1),y_ax_rob_o_quat(2),y_ax_rob_o_quat(3),'color','g')
% quiver3(vicon_point_original(1),vicon_point_original(2),vicon_point_original(3),z_ax_rob_o_quat(1),z_ax_rob_o_quat(2),z_ax_rob_o_quat(3),'color','b')


% % Try Rodrigues Shite
% ax_angle_rob = quat2axang(quat_r);
% 
% k = ax_angle_rob(1:3);
% theta = ax_angle_rob(4);
% 
% W = [0      -k(3)   k(2)
%      k(3)   0       -k(1)
%      -k(2)  k(1)       0];
%  
% R = eye(3) + sin(theta).*W +(1-cos(theta)).*W^2;
% 
% 
% x = [200 0 0];
% y = [0 200 0];
% z = [0 0 200];
% 
% x_ax_rob_o = R*x';
% y_ax_rob_o = R*y';
% z_ax_rob_o = R*z';
% 
% quiver3(vicon_point_original(1),vicon_point_original(2),vicon_point_original(3),x_ax_rob_o(1),x_ax_rob_o(2),x_ax_rob_o(3),'color','r')
% quiver3(vicon_point_original(1),vicon_point_original(2),vicon_point_original(3),y_ax_rob_o(1),y_ax_rob_o(2),y_ax_rob_o(3),'color','g')
% quiver3(vicon_point_original(1),vicon_point_original(2),vicon_point_original(3),z_ax_rob_o(1),z_ax_rob_o(2),z_ax_rob_o(3),'color','b')
% 
% axis equal
% hold off

% straighten pipe and vicon point and shift down to z = 0;

pipe_c_1_stra = pipe_c_1;
pipe_c_1_stra(3) = pipe_c_1_stra(3);
pipe_c_1_stra(1) = pipe_c_1_stra(1) - x_c;  
pipe_c_2_stra = quatrotate(quat_p,pipe_c_2);
pipe_c_2_stra(3) = pipe_c_2_stra(3);
pipe_c_2_stra(1) = pipe_c_2_stra(1) - x_c;

vicon_point_straight = quatrotate(quat_p,vicon_point_original);
vicon_point_straight(3) = vicon_point_straight(3);
vicon_point_straight(1) = vicon_point_straight(1)- x_c;

% figure
% hold on
% h = Cylinder( pipe_c_1_stra, pipe_c_2_stra,253.5,40,'black',0,1,0.1,1)
% 
% scatter3(vicon_point_straight(1), vicon_point_straight(2),vicon_point_straight(3))


% fix robot quaternion
%quat_r_stra = quatmultiply(quat_p_inv,quat_r); %Why doestn this work :( :(


% plot axis given by quaternions

x_ax_rob = quatrotate(quat_p,x_ax_rob_o_quat );
y_ax_rob = quatrotate(quat_p,y_ax_rob_o_quat );
z_ax_rob = quatrotate(quat_p,z_ax_rob_o_quat );


x_axis(1,:) = [0 0 0]+vicon_point_straight;
x_axis(2,:) = x_ax_rob+vicon_point_straight;
y_axis(1,:) = [0 0 0]+vicon_point_straight;
y_axis(2,:) = y_ax_rob+vicon_point_straight;
z_axis(1,:) = [0 0 0]+vicon_point_straight;
z_axis(2,:) = z_ax_rob+vicon_point_straight;

% Shift robot center
x_vec = x_axis(2,:) - x_axis(1,:);
x_vec = x_vec/norm(x_vec);
x_rob_shift = r_c_cor*x_vec;
x_rob_shifted = vicon_point_straight - x_rob_shift;
vicon_point_straight = x_rob_shifted;

%figure
hold on
title('Rotated by Pipe Quaternion')
Cylinder( pipe_c_1_stra, pipe_c_2_stra,253.5,40,'black',0,1,0.1,1)
xlabel('x')
ylabel('y')
zlabel('z')

quiver3(vicon_point_straight(1), vicon_point_straight(2),vicon_point_straight(3), x_ax_rob(1), x_ax_rob(2), x_ax_rob(3),'color', 'r','Linewidth',2)
quiver3(vicon_point_straight(1), vicon_point_straight(2),vicon_point_straight(3), y_ax_rob(1), y_ax_rob(2), y_ax_rob(3),'color', 'g','Linewidth',2)
quiver3(vicon_point_straight(1), vicon_point_straight(2),vicon_point_straight(3), z_ax_rob(1), z_ax_rob(2), z_ax_rob(3),'color', 'b','Linewidth',2)

hold off
    
%% Calculate VICON Omega

 vicon_omega = atan2((vicon_point_straight(3)),(vicon_point_straight(1)));
 vicon_omega_deg = vicon_omega*180/pi;
%% Calcualte VICON Alpha


%% Long ass way to calculate alpha (?)

[vicon_alpha] = VICON_alpha(vicon_data,geo_know, x_axis,vicon_point_straight, vicon_omega) ;
vicon_alpha_deg = vicon_alpha*180/pi;


    %% Plot VICON robot
    
    v_alpha = vicon_alpha;
    v_omega = vicon_omega;

    % Get position of the robot  
    % Get Robot Points
    %% Calcs for forward model
    
    [wheel_1, wheel_2, castor] = get_wheels_vicon_cent(v_alpha, -v_omega+pi/2, geo_know, vicon_point_straight );
    
%      scatter3(wheel_1(1), wheel_1(2), wheel_1(3),'m')
%      scatter3(wheel_2(1), wheel_2(2), wheel_2(3),'m')
%      scatter3(castor(1), castor(2), castor(3),'m')

    
    robot = [wheel_1; wheel_2; castor]; 

%     R = eye(3) + sin(-v_omega+pi/2).*W +(1-cos(-v_omega+pi/2)).*W^2;
% 
%     robot(1,:) =R*robot(1,:)';     
%     robot(2,:) =R*robot(2,:)';
%     robot(3,:) =R*robot(3,:)';
%   
    hold on
    
    
    
    [T, acc_x_sim_vicon, acc_y_sim_vicon, acc_z_sim_vicon, robot_angle] = plot_robot_3d(robot,'k',1);
    
    % Wheel colours
    scatter3(wheel_1(1),wheel_1(2),wheel_1(3),'r','filled')
    scatter3(wheel_2(1),wheel_2(2),wheel_2(3),'r','filled')
    scatter3(castor(1),castor(2),castor(3),'g','filled')
    
    
    accel_sim_vicon = [acc_x_sim_vicon acc_y_sim_vicon acc_z_sim_vicon];

    
    vicon_point = vicon_point_straight;
    
    if vicon_point(1) > 0
        side = sprintf("Right");
        
    elseif vicon_point(1) < 0
        side = sprintf("Left");
    
    else 
        side = sprintf("Top or Bot");
    end
    
 
end
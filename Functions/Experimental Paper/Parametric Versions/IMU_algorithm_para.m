function [ omega_para, alpha_para, omega_algo, alpha_algo, IMU_data_mti] = IMU_algorithm_para(file_mti, geo_know,col,vicon_point)
hold on
%%NOTES:
% w = -w +pi/2 
% This is because the difference in starting position for the calculation 
% ie) top of pipe vs. right side of pipe

r_p = geo_know(1);
b = geo_know(2);
l = geo_know(3);
cylinder_length = geo_know(4);
Y_p = vicon_point(2);
side = geo_know(7);

x_corr = geo_know(8);
y_corr = geo_know(9);
z_corr = geo_know(10);

IMU_mti  = csvread(file_mti ,6, 2);

mti_x = -IMU_mti(:,1);
mti_y = -IMU_mti(:,2);
mti_z = -IMU_mti(:,3);
mti_roll = IMU_mti(:,4);
mti_pitch = IMU_mti(:,5);

IMU_data_mti(1) = mean(mti_x(:,1))+x_corr;
IMU_data_mti(2) = mean(mti_y(:,1))+y_corr;
IMU_data_mti(3) = mean(mti_z(:,1))+z_corr;
IMU_data_mti(4) = mean(mti_roll(:,1));  
IMU_data_mti(5) = mean(mti_pitch(:,1));

x_imu_real = IMU_data_mti(1) ;
y_imu_real = IMU_data_mti(2) ;
z_imu_real = IMU_data_mti(3) ;
roll = IMU_data_mti(4)*pi/180 ;
pitch = -IMU_data_mti(5)*pi/180 ;

%Roll and pitch checks
% 
% roll_check = atan(y_imu_real/z_imu_real);
% pitch_check = atan(x_imu_real/sqrt(y_imu_real^2 + z_imu_real^2));

%% Calcs for forward model

%rot_ax = [0 1 0];
    
W = [    0     -0    1
         0      0   -0
        -1      0    0];
  
    %% Inverse Model  
    
%     Ao = [0;1;0];
%     Bo = [1;0;0];
%     Co = [0;0;0];
    
    % Rotation Matricies

%     Ry = [  cos(pitch)  0   -sin(pitch)
%             0           1   0
%             sin(pitch) 0   cos(pitch)]; %right is +
%         
%         
%     Rx = [  1   0   0
%             0 cos(roll) sin(roll)
%             0   -sin(roll) cos(roll)]; %up is +
% 
%     R_mat = Ry*Rx;
%    
%   yaw_est = atan2(y_imu_real,(x_imu_real));
% 
%     eul = [0  pitch roll];
%     R_mat = eul2rotm(eul);
% 
%     
%     n_dir = [0 0 1];
%     n_dir_rot = n_dir*R_mat;
%     quiver3(vicon_point(1),vicon_point(2),vicon_point(3),n_dir_rot(1),n_dir_rot(2),n_dir_rot(3),100,'color','m')
    

    % Apply to Points     
%     A_o = R_mat*Ao;
%     B_o = R_mat*Bo;
%     C_o = R_mat*Co;
    
    
%     % Create Plane from Points
%     v1 = C_o - B_o;
%     v2 = C_o - A_o;
  
    %Normal Unit Vector
%     n = -cross(v1, v2)/(norm(cross(v1, v2))); 
%     n(3) = -n(3);
%     x_n = n(1);
%     y_n = n(2);
%     z_n = n(3);
    
    %% Plot plane calcuated from roll and pitch of IMU
    %figure
    % Plot PLanes
%     w = null(n'); % Find two orthonormal vectors which are orthogonal to v
%     [P,Q] = meshgrid(-100:100); % Provide a gridwork (you choose the size)
%     X = vicon_point(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
%     Y = vicon_point(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
%     Z = vicon_point(3)+w(3,1)*P+w(3,2)*Q;
%     surf(X,Y,Z)
%     hold on
%     quiver3(vicon_point(1),vicon_point(2),vicon_point(3),n(1), n(2), n(3), 20)
    


    % Alpha and Omega Calculations
    if isequal(side, "Right") == 1 % Right Side
       a = (atan2(y_imu_real,(x_imu_real)))+pi;
    else % Left Side
       a =  atan2(y_imu_real,(x_imu_real));
    end

    % Old method where roll and pitch are used
    %w =  atan2(z_n,sign(x_n)*sqrt(x_n^2 + y_n^2));
    
    w = asin(z_imu_real/9.81)+pi;
    
    %w = w + Const*((sin(a))*(sin(2*a)));
    omega_algo = w;
    alpha_algo = a;
    
    [ wheel_1,wheel_2, castor] = get_wheels_sph(a, -w+pi/2, [r_p, b, l, Y_p]);

    robot = [wheel_1; wheel_2; castor]; 


    [T, acc_x_algo, acc_y_algo, acc_z_algo, robot_angle] = plot_robot_3d(robot,'r',1);

    %% Parametric Compensation
    
    Const_w = 5.57e-01 * exp(-7.86e-03 * r_p) + 1.96e-01 * exp(-1.33e-03 * r_p);

    %Const_w = 9.32e-01 * exp(-1.44e-02 * r_p) + 2.16e-01 * exp(-2.16e-03 * r_p);
    
    w_correction = Const_w*((sin(a))*(sin(2*a)));

    w_corrected = w - w_correction;
    
    w = w_corrected;
    omega_para = w;
    alpha_para = a;
    
    %w = 0;
    [ wheel_1,wheel_2, castor] = get_wheels_sph(a, -w+pi/2, [r_p, b, l, Y_p]);
    
    robot = [wheel_1; wheel_2; castor]; 

    
    [T, acc_x_algo, acc_y_algo, acc_z_algo, robot_angle] = plot_robot_3d(robot,'g',1);

    x_imu_algo(1,:) = acc_x_algo;
    y_imu_algo(1,:) = acc_y_algo;
    z_imu_algo(1,:) = acc_z_algo;

   
    %% Plot optimised robot


    %hold off;
   
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    
    xlim([-r_p-150 r_p+150])
    ylim([vicon_point(2)-cylinder_length/2 vicon_point(2)+cylinder_length/2])
    zlim([-r_p-150 r_p+150])
    daspect([1 1 1])
    %view([0 0 1])
    drawnow

 
end
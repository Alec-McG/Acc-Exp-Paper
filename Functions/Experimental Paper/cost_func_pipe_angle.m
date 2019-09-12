function res = cost_func_pipe_angle(x, acc_x, acc_y, acc_z, r_p,b,l,Y_p, pipe_angle)

    geo(1) = r_p;
    geo(2) = b; 
    geo(3) = l;
    geo(4) = Y_p;
    
    rot_ax_pipe = [1 0 0];
    
    
    W_pipe = [    0     -rot_ax_pipe(3)    rot_ax_pipe(2)
     rot_ax_pipe(3)      0   -rot_ax_pipe(1)
    -rot_ax_pipe(2)      rot_ax_pipe(1)    0];
    
    
    a_opt = x(1);
    w_opt = x(2);

    [wheel_1_opt, wheel_2_opt, castor_opt] = get_wheels_sph(a_opt, w_opt, geo );

    robot_opt = [wheel_1_opt; wheel_2_opt; castor_opt]; 
    
    R_pipe = eye(3) + sin(pipe_angle).*W_pipe +(1-cos(pipe_angle)).*W_pipe^2;

    robot_opt(1,:) =R_pipe*robot_opt(1,:)';     
    robot_opt(2,:) =R_pipe*robot_opt(2,:)';
    robot_opt(3,:) =R_pipe*robot_opt(3,:)';
    
   
    [T, acc_x_opt, acc_y_opt, acc_z_opt, robot_angle] = plot_robot_3d(robot_opt, 'g', 0);
    

    res = (acc_x_opt - acc_x)^2 + (acc_y_opt - acc_y)^2 + (acc_z_opt - acc_z)^2;

%% Optimise Ratios

%     geo(1) = r_p;
%     geo(2) = b; 
%     geo(3) = l;
%     geo(4) = Y_p;
%     
%     a_opt = x(1);
%     w_opt = x(2);
% 
%     [l_check, wheel_1_opt, wheel_2_opt, castor_opt] = get_wheels(a_opt, w_opt, geo );
% 
%     robot_opt = [wheel_1_opt; wheel_2_opt; castor_opt]; 
%    
%     [T, acc_x_opto, acc_y_opto, acc_z_opto, robot_angle] = plot_robot_3d(robot_opt, 'g', 0);
%     
%     acc_x_opto=acc_x_opto;
%     acc_y_opto=acc_y_opto;
%     acc_z_opto=acc_z_opto;
%     
%     ratio_1_opt = acc_x_opto^2/sqrt(acc_x_opto^2 +acc_y_opto^2+acc_z_opto^2);
%     ratio_2_opt = acc_y_opto^2/sqrt(acc_x_opto^2 +acc_y_opto^2+acc_z_opto^2);
%     ratio_3_opt = acc_z_opto^2/sqrt(acc_x_opto^2 +acc_y_opto^2+acc_z_opto^2);
%     
%     ratio_1_real = acc_x^2/sqrt(acc_x^2 +acc_y^2+acc_z^2);
%     ratio_2_real = acc_y^2/sqrt(acc_x^2 +acc_y^2+acc_z^2);
%     ratio_3_real = acc_z^2/sqrt(acc_x^2 +acc_y^2+acc_z^2);
%     
   
 %   res = (ratio_1_opt - ratio_1_real)^2 + (ratio_2_opt - ratio_2_real)^2 + (ratio_3_opt - ratio_3_real)^2;
    
    
    
    
    
    
    
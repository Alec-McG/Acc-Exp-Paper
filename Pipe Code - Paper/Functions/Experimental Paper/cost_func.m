function res = cost_func(x, acc_x, acc_y, acc_z, r_p,b,l,Y_p, g_imu)

    geo(1) = r_p;
    geo(2) = b; 
    geo(3) = l;
    geo(4) = Y_p;
    
    a_opt = x(1);
    w_opt = x(2);

    [wheel_1_opt, wheel_2_opt, castor_opt] = get_wheels_sph(a_opt, w_opt, geo );

    robot_opt = [wheel_1_opt; wheel_2_opt; castor_opt]; 
   
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
    
    
    
    
    
    
    
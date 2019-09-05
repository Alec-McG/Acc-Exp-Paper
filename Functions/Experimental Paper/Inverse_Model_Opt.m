function [robot_inv, acc_inv, angles_inv, robot_opt, acc_opt, angles_opt ] = Inverse_Model_Opt(acc_x, acc_y, acc_z, geo, plot)


    r_p = geo(1);
    b = geo(2); 
    l = geo(3);
    Y_p = geo(4);
    side = geo(5);
    
    if (side == 0)
        side = "Right";
    else
        side  = "Left";
    end

    
    %% Inverse Model
    if isequal(side, "Right") == 1 % Right Side
       a = (atan2(acc_y,(acc_x))+pi);
       w = acos(acc_z/-9.81);
    else % Left Side
       a =  atan2(acc_y,(acc_x));
       w = -acos(acc_z/-9.81);
    end
    
    angles_inv = [a,w];
    
    
    
    
    %[ wheel_1_inv,wheel_2_inv, castor_inv] = get_wheels_sph(a, -w+pi/2, [r_p, b, l, Y_p]);
    
    [ wheel_1_inv,wheel_2_inv, castor_inv] = get_wheels_sph(a, w, [r_p, b, l, Y_p]);
    
    robot_inv = [wheel_1_inv; wheel_2_inv; castor_inv]; 
    
    [T, acc_x_inv, acc_y_inv, acc_z_inv, robot_angle] = plot_robot_3d(robot_inv,'r',plot);

    acc_inv = [acc_x_inv, acc_y_inv, acc_z_inv];
    
    %% Optimisation
    
    %x0 = [a    -w+pi/2];
    x0 = [a    w];
    
    f = @(x)cost_func(x, acc_x, acc_y, acc_z, r_p,b,l,Y_p);
    
    
    [an, fval] = lsqnonlin(f, x0,[],[]);
    
    a_opt = an(1);
    w_opt = an(2);
    
    angles_opt = [a_opt, w_opt];

    %[ wheel_1_opt,wheel_2_opt, castor_opt] = get_wheels_sph(a_opt, -w_opt+pi/2, [r_p, b, l, Y_p]);
    [ wheel_1_opt,wheel_2_opt, castor_opt] = get_wheels_sph(a_opt, w, [r_p, b, l, Y_p]);
    
    robot_opt = [wheel_1_opt; wheel_2_opt; castor_opt];
    
    [T, acc_x_opt, acc_y_opt, acc_z_opt, robot_angle] = plot_robot_3d(robot_opt,'m',plot);

    acc_opt = [acc_x_opt, acc_y_opt, acc_z_opt];
    
    
    
    
end 
function [wheels, acc] = Forward_Model_Angle(alpha, omega, pipe_angle, geo, plot)

    % Rotation for w angle
    rot_ax = [0 1 0];

    W_w = [    0     -rot_ax(3)    rot_ax(2)
         rot_ax(3)      0   -rot_ax(1)
        -rot_ax(2)      rot_ax(1)    0];


    % Rotation for angled pipe

    rot_ax_pipe = [1 0 0];

    W_pipe = [    0     -rot_ax_pipe(3)    rot_ax_pipe(2)
         rot_ax_pipe(3)      0   -rot_ax_pipe(1)
        -rot_ax_pipe(2)      rot_ax_pipe(1)    0];


    [wheel_1, wheel_2, castor] = get_wheels_sph(alpha, omega, geo);
    
    robot = [wheel_1; wheel_2; castor]; 

    R_pipe = eye(3) + sin(pipe_angle).*W_pipe +(1-cos(pipe_angle)).*W_pipe^2;

    robot(1,:) =R_pipe*robot(1,:)';     
    robot(2,:) =R_pipe*robot(2,:)';
    robot(3,:) =R_pipe*robot(3,:)';

    [T, acc_x, acc_y, acc_z, robot_angle] = plot_robot_3d(robot,'g',plot);

    
    wheels = [wheel_1; wheel_2; castor];
    acc = [acc_x, acc_y acc_z];
end
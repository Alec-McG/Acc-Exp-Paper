function [T, Rx, Ry, Rz, robot_angle] = plot_robot_3d(p, co, do_plot)



if(~exist('co','var'))
    co = 'b';
end

if(~exist('do_plot','var'))
    do_plot = 0;
end

% Plot the wheels
if(do_plot == 1)
    
    hold on

    fmt = {'Fontsize' 12, 'Fontweight', 'b'};

    % Plot wheel points
    %plot3(p(:,1), p(:,2), p(:,3),'.', 'color',co, 'Markersize', 20);
end

% Plot the chassis
R = [p
     p(1,:)];
if(do_plot == 1) 
    plot3(R(:,1), R(:,2), R(:,3), 'color', co, 'linewidth',1);
end
wheel_1 = p(1,:);
wheel_2 = p(2,:);
castor =  p(3,:);

%% Get the position of the origin on the wheel plane
[c, wheel_plane_normal] = compute_plane_coefficients(p');
wheel_plane_midpoint= 0.5*(wheel_1 + wheel_2);
wheel_plane_normal = -wheel_plane_normal;

%% Get the position of the origin on the top plate
wheel_1_height = 60;
wheel_2_height = 60;
castor_height  = 60;

p_1(1,:) = wheel_1;
p_1(2,:) = wheel_1 + wheel_1_height*wheel_plane_normal;
p_2(1,:) = wheel_2;
p_2(2,:) = wheel_2 + wheel_2_height*wheel_plane_normal;
p_3(1,:) = castor;
p_3(2,:) = castor + castor_height*wheel_plane_normal;
plate = [p_1(2,:)
         p_2(2,:)
         p_3(2,:)
         p_1(2,:)]; 
[c, chassis_plane_normal] = compute_plane_coefficients(plate(1:3,:)');
chassis_plane_midpoint = 0.5*(p_1(2,:)+ p_2(2,:));
plate_normal(1,:) = chassis_plane_midpoint;
plate_normal(2,:) = chassis_plane_midpoint + 5*chassis_plane_normal;

if(do_plot == 1)
    plot3(p_1(:,1), p_1(:,2), p_1(:,3),'color', co);
    plot3(p_2(:,1), p_2(:,2), p_2(:,3),'color', co);
    plot3(p_3(:,1), p_3(:,2), p_3(:,3),'color', co);
    plot3(plate(:,1),plate(:,2), plate(:,3),'color', co);
end
%% Compute axes of the coordinate system

midpoint = chassis_plane_midpoint;
n        = chassis_plane_normal;
X        = p_1(2,:);

n = -n;

% midpoint = wheel_plane_midpoint;
% n        = wheel_plane_normal;
% X        = wheel_2;

% Create Y axis

ss = 100;
y_axis(1,:) = midpoint;
y_axis(2,:) = X;
y_vec = diff(y_axis);
y_vec = y_vec/norm(y_vec);
y_axis(2,:) = midpoint+y_vec*ss;



% Fix Z axis at the midpoint of the drive wheels
z_axis(1,:) = midpoint;
z_axis(2,:) = midpoint + ss*n;
z_vec = diff(z_axis);
z_vec = z_vec/norm(z_vec);

% Create X axis
x_vec = cross(y_vec,z_vec);
x_vec = x_vec/norm(x_vec);
x_axis(1,:) = midpoint;
x_axis(2,:) = midpoint + ss*x_vec;

T = [x_vec.' y_vec.' z_vec.' midpoint.'];

if(do_plot == 1)
    
    
    % X axis
    plot3(x_axis(:,1), x_axis(:,2), x_axis(:,3),'color', co, 'linewidth',2);
    %text(x_axis(2,1), x_axis(2,2), x_axis(2,3), 'X_{IMU}','color','red',  fmt{:});
    
    % Y axis
    plot3(y_axis(:,1), y_axis(:,2), y_axis(:,3), 'color', co, 'linewidth',2);
    %text(y_axis(2,1), y_axis(2,2), y_axis(2,3), 'Y_{IMU}','color','red', fmt{:});
    
    % Z axis
    plot3(z_axis(:,1), z_axis(:,2), z_axis(:,3), 'color', co, 'linewidth',2);
    %text(z_axis(2,1), z_axis(2,2), z_axis(2,3), 'Z_{IMU}','color','red',  fmt{:});
end

% Angle of the line connect the wheels w.r.t the main axis (Y-Axis) of the pipe
dx = wheel_2(2) - wheel_1(2);
dy = wheel_2(3) - wheel_1(3);
robot_angle = atan2(dy, dx)*180/pi;


%Compute the output of the accelerometer
[Rx, Ry, Rz] = get_accelerometer(T);

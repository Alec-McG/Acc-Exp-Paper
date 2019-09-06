function [Rx, Ry, Rz] = get_accelerometer_error(T, grav_error)

x_vec = T(1:3,1);
y_vec = T(1:3,2);
z_vec = T(1:3,3);

% Angle of the gravity vector with respect to each axis
Axr = angle_between_vectors([0 0 -1], x_vec);
Ayr = angle_between_vectors([0 0 -1], y_vec);
Azr = angle_between_vectors([0 0 -1], z_vec);

R = 9.81; 
Rx = R*cos(Axr)+(R*grav_error);
Ry = R*cos(Ayr);
Rz = R*cos(Azr);

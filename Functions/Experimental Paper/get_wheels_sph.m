%% Get Wheel Position Coordinates Function

function [wheel_1, wheel_2, castor] = get_wheels_sph(alpha, omega, geo )

    % Set Variables
    
    r_p = geo(1);
    b = geo(2); 
    l = geo(3);
    Y_p = geo(4);
    
    
    W = [    0     -0    1
         0      0   -0
        -1      0    0];
    
    %% Calculations

    % Points at alpha 0 for calculations
    
    Y_D1_0 = (b/2)*cos(0+(180)*pi/180);
    X_D1_0 = (b/2)*sin(0+(180)*pi/180);
    Z_D1_0 = sqrt(abs(r_p^2 - X_D1_0^2));

    Y_D2_0 = (b/2)*cos(0+(0)*pi/180);
    X_D2_0 = (b/2)*sin(0+(0)*pi/180);
    Z_D2_0 = sqrt(abs(r_p^2 - X_D2_0^2));
    
    
      
    rc = r_p; 
    Rs = l;
      
    ang = 0.001;
    a = Z_D1_0;
    

    theta = atan2(0.1e1 / Rs / sin(ang) ^ 2 * sqrt(0.2e1 * a * sqrt(Rs ^ 2 * sin(ang) ^ 4 + (-Rs ^ 2 - a ^ 2 + rc ^ 2) * sin(ang) ^ 2 + a ^ 2) + (Rs ^ 2 + a ^ 2 - rc ^ 2) * sin(ang) ^ 2 - 0.2e1 * a ^ 2), (-a + sqrt(Rs ^ 2 * sin(ang) ^ 4 + (-Rs ^ 2 - a ^ 2 + rc ^ 2) * sin(ang) ^ 2 + a ^ 2)) / sin(ang) ^ 2 / Rs);

    %theta = atan2(0.1e1 / sin(ang) ^ 2 * sqrt(sin(ang) ^ 2 * Rs ^ 2 + sin(ang) ^ 2 * a ^ 2 - sin(ang) ^ 2 * rc ^ 2 + 0.2e1 * a * sqrt(sin(ang) ^ 4 * Rs ^ 2 - sin(ang) ^ 2 * Rs ^ 2 - sin(ang) ^ 2 * a ^ 2 + sin(ang) ^ 2 * rc ^ 2 + a ^ 2) - 0.2e1 * a ^ 2) / Rs, (-a + sqrt(sin(ang) ^ 4 * Rs ^ 2 - sin(ang) ^ 2 * Rs ^ 2 - sin(ang) ^ 2 * a ^ 2 + sin(ang) ^ 2 * rc ^ 2 + a ^ 2)) / sin(ang) ^ 2 / Rs);

    
    X_sph_f = Rs*cos(ang).*sin(theta);
    Y_sph_f = Rs*sin(ang).* sin(theta)+Y_p;
    Z_sph_f = Rs*cos(theta)+ a;
    
    X_C_0 = X_sph_f; 
    Y_C_0 = Y_sph_f;
    Z_C_0 = Z_sph_f;
    
   %  scatter3(X_sph_f,Y_sph_f,Z_sph_f,'r')
   %  drawnow
    

     wheel_1_0 = [X_D1_0 Y_D1_0 + Y_p  Z_D1_0 ];
     wheel_2_0 = [X_D2_0 Y_D2_0 + Y_p  Z_D2_0 ];
     
    
    
    castor_0 =  [X_C_0  Y_C_0    Z_C_0 ];
    
    centre_0 = (((wheel_1_0+ wheel_2_0)/2) + castor_0)/2;
    
%     hold on
%     scatter3(wheel_1_0(1), wheel_1_0(2), wheel_1_0(3))
%     scatter3(wheel_2_0(1), wheel_2_0(2), wheel_2_0(3))
%     scatter3(castor_0(1), castor_0(2), castor_0(3))
    
    % Alpha Positions
    
    Y_D1 = (b/2)*cos(alpha+(180)*pi/180);
    X_D1 = (b/2)*sin(alpha+(180)*pi/180);
    Z_D1 = sqrt(abs(r_p^2 - X_D1^2));

    Y_D2 = (b/2)*cos(alpha+(0)*pi/180);
    X_D2 = (b/2)*sin(alpha+(0)*pi/180);
    Z_D2 = sqrt(abs(r_p^2 - X_D2^2));


    a = Z_D1;
    
    theta = atan2(0.1e1 / Rs / sin(-alpha) ^ 2 * sqrt(0.2e1 * a * sqrt(Rs ^ 2 * sin(-alpha) ^ 4 + (-Rs ^ 2 - a ^ 2 + rc ^ 2) * sin(-alpha) ^ 2 + a ^ 2) + (Rs ^ 2 + a ^ 2 - rc ^ 2) * sin(-alpha) ^ 2 - 0.2e1 * a ^ 2), (-a + sqrt(Rs ^ 2 * sin(-alpha) ^ 4 + (-Rs ^ 2 - a ^ 2 + rc ^ 2) * sin(-alpha) ^ 2 + a ^ 2)) / sin(-alpha) ^ 2 / Rs);

    X_sph = Rs*cos(-alpha).*sin(theta);
    Y_sph = Rs*sin(-alpha).*sin(theta);
    Z_sph = Rs*cos(theta)+ a;

    X_C = X_sph; 
    Y_C = Y_sph;
    Z_C = Z_sph;
    

    wheel_1 = [X_D1 Y_D1 + Y_p  Z_D1 ];
    wheel_2 = [X_D2 Y_D2 + Y_p  Z_D2 ];
    castor =  [X_C  Y_C  + Y_p   Z_C ];
    
%     hold on
%     scatter3(wheel_1(1), wheel_1(2), wheel_1(3))
%     scatter3(wheel_2(1), wheel_2(2), wheel_2(3))
%     scatter3(castor(1), castor(2), castor(3))
    
    robot = [wheel_1; wheel_2; castor]; 
    
    centre = (((wheel_1+ wheel_2)/2) + castor)/2;
  
    % Rotate by Omega
    
   %% COPY THIS FOR THE VICON ALPHA CALCUALTIONS!!!!
    %w_first = -asin(centre(1)/r_p);
     
    w_first = -atan(centre(1)/centre(3)); 
    
    R_w_first = eye(3) + sin(w_first).*W +(1-cos(w_first)).*W^2;
    
    robot(1,:) =R_w_first*robot(1,:)';     
    robot(2,:) =R_w_first*robot(2,:)';
    robot(3,:) =R_w_first*robot(3,:)';
    
    %scatter3(robot(:,1), robot(:,2), robot(:,3), 'r','filled')
    
    R_sim = eye(3) + sin(omega).*W +(1-cos(omega)).*W^2;

    robot(1,:) =R_sim*robot(1,:)';     
    robot(2,:) =R_sim*robot(2,:)';
    robot(3,:) =R_sim*robot(3,:)';
    
    %robot(:,2) = robot(:,2) - (centre(2)-Y_p); 
    
    % Shift centre
    
    cent_shift = centre - centre_0;
    
    wheel_1 =  robot(1,:);
    wheel_2 = robot(2,:);
    castor = robot(3,:);
    
    wheel_1(2) = wheel_1(2) - cent_shift(2);
    wheel_2(2) = wheel_2(2) - cent_shift(2);
    castor(2) = castor(2) - cent_shift(2);

    robot(1,:) = wheel_1;
    robot(2,:) = wheel_2;
    robot(3,:) = castor;
    
    %scatter3(robot(:,1), robot(:,2), robot(:,3), 'o','filled')
    
    %scatter3(robot(:,1), robot(:,2), robot(:,3), 'm','filled')
    
    %Checks
    
    %drive_cent = (wheel_1 + wheel_2)/2;
    %vec = drive_cent - castor;
    %l_check = sqrt( vec(1)^2 + vec(2)^2 + vec(3)^2)
    
end
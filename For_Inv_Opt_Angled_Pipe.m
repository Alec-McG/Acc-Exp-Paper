%% Angled Pipes Forward Model
%

%% 
close all
clear all
filename = '0_deg_opt.gif';
folder = '/GIFS';
%% Pipe Parameters
do_plot = 1;
r_p         = 250;
pipe_length = 500;
% Create Cylinder
pipe_angle = 10*pi/180;

pipe_centre = [0  0 0];
%pipe_centre2 = [0 pipe_length*cos(pipe_angle) pipe_length*sin(pipe_angle)];
pipe_centre2 = [0 pipe_length pipe_length*sin(pipe_angle)];
%% Robot Parameters
b   = 100;
l   = 125; 
Y_p = 0.5*pipe_length;
% side = "Right";
% 
% if isequal(side, "Right")
%     side = 0;
%     
% else
%     side  = 1;
% end
%% Angles
%W_sim = linspace(1,355,60);
A_sim = linspace(5,350,100);
%A_sim(1:length(W_sim))  = 30;
W_sim(1:length(A_sim)) = 135;

fig1 = figure;
set(fig1, 'Position', [100, 100, 1000, 900])


    geo(1) = r_p;
    geo(2) = b; 
    geo(3) = l;
    geo(4) = Y_p;
    geo(6) = pipe_angle;

    
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
    
        
        
for i = 1:length(A_sim) 
    clf
        
    a_sim = A_sim(i)*pi/180;
    w_sim = W_sim(i)*pi/180;
    
    % Right and Left side of pipe - Assuming known
    
    if (1 < W_sim(i) && W_sim(i) < 179 ) %Right Side
        side = 0;
        
    elseif (-179< W_sim(i) && W_sim(i) < -1 || 181 < W_sim(i) && W_sim(i) < 359 ) %Left Side
        side = 1;
        
    else % Top or Bottom 
        side = 0;        
        
    end
    
    geo(5) = side;

    % Forward Model
    
    [wheel_sim, acc_sim] = Forward_Model_Angle(a_sim, w_sim, pipe_angle, geo, do_plot);
       
    acc_x = acc_sim(1);
    acc_y = acc_sim(2);
    acc_z = acc_sim(3);
    
    hold on   
    
    % Inverse Model + Optimisation - (Assuming Horizontal Pipe)
    
    [robot_inv, acc_inv, angles_inv, robot_opt, acc_opt, angles_opt ] = Inverse_Model_Opt_Angled_Pipe(acc_x, acc_y, acc_z, geo, do_plot);
    
    % Plot Inverse Model/Opt
    
    
    
    % Plot Flat Cylinder
    
    h = Cylinder(pipe_centre, pipe_centre2,r_p,40,'black',0,1,0.8,1);
      
%     x_imu_sim(i,:) = acc_x;
%     y_imu_sim(i,:) = acc_y;
%     z_imu_sim(i,:) = acc_z;

    xlim([-r_p-300 r_p+300])
    ylim([0-300 pipe_length+300])
    zlim([-r_p-300 r_p+300])
    daspect([1 1 1])
    view([-1 -1 1])

    drawnow

    
    %% Save Variables for Plot
    
    acc_sim_plot(i,:) = acc_sim;
    acc_inv_plot(i,:) = acc_inv;
    acc_opt_plot(i,:) = acc_opt;
    
    angles_sim_plot(i,:) = [a_sim, w_sim];
    angles_inv_plot(i,:) = angles_inv;
    angles_opt_plot(i,:) = angles_opt;
    
    
    
    
end


%% Calculate Errors

inv_acc_error = (acc_sim_plot - acc_inv_plot);
opt_acc_error = (acc_sim_plot - acc_opt_plot);

inv_angle_error = (angles_sim_plot - angles_inv_plot);
opt_angle_error = (angles_sim_plot - angles_opt_plot);


%% Plot Acceleration Errors

figure
hold on
subplot(3,1,1)
erplt_accx = plot(angles_sim_plot(:,1),inv_acc_error(:,1),angles_sim_plot(:,1),opt_acc_error(:,1));
xlabel('Simulated Alpha')
ylabel('Acc X Error')
title(' Acc X Error')
legend('Inverse Error','Optimisation Error')

subplot(3,1,2)
erplt_accy = plot(angles_sim_plot(:,1),inv_acc_error(:,2),angles_sim_plot(:,1),opt_acc_error(:,2));
xlabel('Simulated Y Acc')
ylabel('Acc Y Error')
title(' Acc Y Error')
legend('Inverse Error','Optimisation Error')

subplot(3,1,3)
erplt_accz = plot(angles_sim_plot(:,1),inv_acc_error(:,3),angles_sim_plot(:,1),opt_acc_error(:,3));
xlabel('Simulated Z Acc')
ylabel('Acc Z Error')
title(' Acc Z Error')
legend('Inverse Error','Optimisation Error')


set([erplt_accx , erplt_accy ,erplt_accz ],'LineWidth',2)

%% Plot Angle Errors

figure
hold on
subplot(2,1,1)
erplt_alpha = plot(angles_sim_plot(:,1),wrapToPi(inv_angle_error(:,1)),angles_sim_plot(:,1),wrapToPi(opt_angle_error(:,1)));
xlabel('Simulated Alpha Angle')
ylabel('Alpha Error')
title(' Alpha Error')
legend('Inverse Error','Optimisation Error')

subplot(2,1,2)
erplt_omega = plot(angles_sim_plot(:,1),wrapToPi(inv_angle_error(:,2)),angles_sim_plot(:,1),wrapToPi(opt_angle_error(:,2)));
xlabel('Simulated Alpha Angle')
ylabel('Omega Error')
title(' Omega Error')
legend('Inverse Error','Optimisation Error')


set([erplt_alpha , erplt_omega ],'LineWidth',2)





%% Angled Pipes Forward Model
%

%% 
close all
clear all
filename = '0_deg_opt.gif';
folder = '/GIFS';
%% Pipe Parameters
r_p         = 250;
pipe_length = 500;
% Create Cylinder
pipe_angle = linspace(0,5,6);

pipe_centre = [0  0 0];
%pipe_centre2 = [0 pipe_length*cos(pipe_angle) pipe_length*sin(pipe_angle)];
pipe_centre2 = [0 pipe_length 0];
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
A_sim = linspace(1,359,100);
%A_sim(1:length(W_sim))  = 30;
W_sim(1:length(A_sim)) = 135;

% fig1 = figure;
% set(fig1, 'Position', [100, 100, 1000, 900])


    geo(1) = r_p;
    geo(2) = b; 
    geo(3) = l;
    geo(4) = Y_p;

    
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
    
plot_bot = 0;

for ii = 1:length(pipe_angle)
    for i = 1:length(A_sim) 
        


        a_sim = A_sim(i)*pi/180;
        w_sim = W_sim(i)*pi/180;
        pipe_ang = pipe_angle(ii)*pi/180;

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

        [wheel_sim, acc_sim] = Forward_Model_Angle(a_sim, w_sim, pipe_ang, geo, plot_bot);

        acc_x = acc_sim(1);
        acc_y = acc_sim(2);
        acc_z = acc_sim(3);

        %hold on   

        % Inverse Model + Optimisation - (Assuming Horizontal Pipe)

        [robot_inv, acc_inv, angles_inv, robot_opt, acc_opt, angles_opt ] = Inverse_Model_Opt(acc_x, acc_y, acc_z, geo, plot_bot);

        % Plot Inverse Model/Opt



        % Plot Flat Cylinder

        %h = Cylinder(pipe_centre, pipe_centre2,r_p,40,'black',0,1,0.8,1);

%          figure(1)
%         clf
%         xlim([-r_p-300 r_p+300])
%         ylim([0-300 pipe_length+300])
%         zlim([-r_p-300 r_p+300])
%         daspect([1 1 1])
%         view([-1 -1 1])
% 
%         drawnow


        %% Save Variables for Plot

        acc_sim_plot(i,:) = acc_sim;
        acc_inv_plot(i,:) = acc_inv;
        acc_opt_plot(i,:) = acc_opt;

        angles_sim_plot(i,:) = [a_sim, w_sim];
        angles_inv_plot(i,:) = angles_inv;
        angles_opt_plot(i,:) = angles_opt;




    end


    %% Calculate Errors

    inv_acc_error = abs(acc_sim_plot - acc_inv_plot)*180/pi;
    opt_acc_error = abs(acc_sim_plot - acc_opt_plot)*180/pi;

    inv_angle_error = abs(angles_sim_plot - angles_inv_plot)*180/pi;
    opt_angle_error = abs(angles_sim_plot - angles_opt_plot)*180/pi;


    %% Plot Acceleration Errors

%     figure(1)
%     subplot(3,1,1)
%     hold on
%     erplt_accx = plot(angles_sim_plot(:,1),inv_acc_error(:,1),angles_sim_plot(:,1),opt_acc_error(:,1));
%     xlabel('Simulated Alpha')
%     ylabel('Acc X Error')
%     title('ABS Acc X Error')
%     legend('Inverse Error','Optimisation Error')
% 
%     subplot(3,1,2)
%     hold on
%     erplt_accy = plot(angles_sim_plot(:,1),inv_acc_error(:,2),angles_sim_plot(:,1),opt_acc_error(:,2));
%     xlabel('Simulated Y Acc')
%     ylabel('Acc Y Error')
%     title('ABS Acc Y Error')
%     legend('Inverse Error','Optimisation Error')
% 
%     subplot(3,1,3)
%     hold on
%     erplt_accz = plot(angles_sim_plot(:,1),inv_acc_error(:,3),angles_sim_plot(:,1),opt_acc_error(:,3));
%     xlabel('Simulated Z Acc')
%     ylabel('Acc Z Error')
%     title('ABS Acc Z Error')
%     legend('Inverse Error','Optimisation Error')
% 
% 
%     set([erplt_accx , erplt_accy ,erplt_accz ],'LineWidth',2)

    %% Plot Angle Errors

    figure(2)
    subplot(2,1,1)
    hold on
%     plot(angles_sim_plot(:,1),abs(wrapToPi(inv_angle_error(:,1)))*180/pi, 'Color', [0.5 0 0] + (ii)/ (2*max(pipe_angle)), "linewidth", 2)
    plot(angles_sim_plot(:,1),abs(wrapToPi(opt_angle_error(:,1))), "linewidth", 2);
    xlabel('Simulated Alpha Angle')
    ylabel('Alpha Error')
    title('ABS Alpha Error')
%     legend('Inverse Error','Optimisation Error')

    subplot(2,1,2)
    hold on
%     plot(angles_sim_plot(:,1),abs(wrapToPi(inv_angle_error(:,2)))*180/pi, 'Color', [0.5 0 0] + (ii)/ (2*max(pipe_angle)), "linewidth", 2)
    plot(angles_sim_plot(:,1),abs(wrapToPi(opt_angle_error(:,2))), "linewidth", 2);
    xlabel('Simulated Alpha Angle')
    ylabel('Omega Error')
    title('ABS Omega Error')
%     legend('Inverse Error','Optimisation Error')


%    set([erplt_alpha , erplt_omega ],'LineWidth',2)

    
    
    
    
    
    
    
end
        
figure(2)
    subplot(2,1,1)
         xlim([0 2*pi]);    
    legend(  sprintf('Pipe Angle: %d', pipe_angle(1)), sprintf('Pipe Angle: %d', pipe_angle(2)),...
             sprintf('Pipe Angle: %d', pipe_angle(3)), sprintf('Pipe Angle: %d', pipe_angle(4)),...
             sprintf('Pipe Angle: %d', pipe_angle(5)), sprintf('Pipe Angle: %d', pipe_angle(6)),...
             'location', 'eastoutside')


         
    subplot(2,1,2)
    xlim([0 2*pi]);
     
    legend(  sprintf('Pipe Angle: %d', pipe_angle(1)), sprintf('Pipe Angle: %d', pipe_angle(2)),...
             sprintf('Pipe Angle: %d', pipe_angle(3)), sprintf('Pipe Angle: %d', pipe_angle(4)), ...
             sprintf('Pipe Angle: %d', pipe_angle(5)), sprintf('Pipe Angle: %d', pipe_angle(6)),...
             'location', 'eastoutside')






%% Plots WRT OMEGA
%line = linspace(0,1,length(A_sim));

% Angles
% figure
% subplot(2,1,1)
% plot1 = plot(W_sim,all_a(:,1),'-o',W_sim,all_a(:,2),':s',W_sim, all_a(:,3),'-.*','markers',3);
% title('Alpha vs. Omega')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Alphas [°]')
% 
% subplot(2,1,2)
% plot2 = plot(W_sim,all_w(:,1),'-o',W_sim,all_w(:,2),':s',W_sim, all_w(:,3),'-.*','markers',3);
% title('Omega vs. Omega')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Omegas [°]')
% 
% set([plot1, plot2],'LineWidth',2)
% % Accelerometer 
% figure
% subplot(3,1,1)
% plot3 = plot(W_sim,x_imu_sim,'-o',W_sim,x_imu_algo,':s',W_sim, x_imu_opt,'-.*','markers',3);
% title('X ACCEL')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Acc [mm/s^2]')
% 
% subplot(3,1,2)
% plot4 = plot(W_sim,y_imu_sim,'-o',W_sim,y_imu_algo,':s',W_sim, y_imu_opt,'-.*','markers',3);
% title('Y ACCEL')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Acc [mm/s^2]')
% 
% subplot(3,1,3)
% plot5 = plot(W_sim,z_imu_sim,'-o',W_sim,z_imu_algo,':s',W_sim, z_imu_opt,'-.*','markers',3);
% title('Z ACCEL')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Acc [mm/s^2]')
% 
% set([plot3, plot4, plot5],'LineWidth',2)
%% Plots WRT ALPHA

% figure
% subplot(2,1,1)
% plot1 = plot(A_sim,all_a(:,1),'-o',A_sim,all_a(:,2),':s',A_sim, all_a(:,3),'-.*','markers',3);
% title('Alpha vs. Omega')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Alpha [°]')
% ylabel('Alphas [°]')
% 
% subplot(2,1,2)
% plot2 = plot(A_sim,all_w(:,1),'-o',A_sim,all_w(:,2),':s',A_sim, all_w(:,3),'-.*','markers',3);
% title('Omega vs. Omega')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Omegas [°]') 

% set([plot1, plot2],'LineWidth',2)
% % Accelerometer 
% figure
% subplot(3,1,1)
% plot3 = plot(A_sim,x_imu_sim,'-o',A_sim,x_imu_algo,':s',A_sim, x_imu_opt,'-.*','markers',3);
% title('X ACCEL')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Acc [mm/s^2]')

% subplot(3,1,2)
% plot4 = plot(A_sim,y_imu_sim,'-o',A_sim,y_imu_algo,':s',A_sim, y_imu_opt,'-.*','markers',3);
% title('Y ACCEL')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Acc [mm/s^2]')

% subplot(3,1,3)
% plot5 = plot(A_sim,z_imu_sim,'-o',A_sim,z_imu_algo,':s',A_sim, z_imu_opt,'-.*','markers',3);
% title('Z ACCEL')
% legend('Simulated','Algorithm','Optimisation')
% xlabel('Simulated Omega [°]')
% ylabel('Acc [mm/s^2]')

% set([plot3, plot4, plot5],'LineWidth',2)
%% Angled Pipes Forward Model
%

%% 
close all
clear all
filename = '0_deg_opt.gif';
folder = '/GIFS';
%% Pipe Parameters
r_p             = 250;
pipe_length = 500;
% Create Cylinder
pipe_angle = 30*pi/180;

pipe_centre = [0  0 0];
pipe_centre2 = [0 pipe_length*cos(pipe_angle) pipe_length*sin(pipe_angle)];
%% Robot Parameters
b   = 100;
l   = 125; 
Y_p = 0.5*pipe_length;
%% Angles
W_sim = linspace(1,179,30);
%A_sim = linspace(1,359,100);
A_sim(1:length(W_sim))  = 90;
%W_sim(1:length(A_sim)) = 45;

fig1 = figure;
set(fig1, 'Position', [100, 100, 1000, 900])


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
    

for i = 1:length(A_sim) 
    clf
        
    a_sim = A_sim(i)*pi/180;
    w_sim = W_sim(i)*pi/180;

    % Get Wheel Positions
    
    [wheel_1_sim, wheel_2_sim, castor_sim, acc_x, acc_y, acc_z] = Forward_Model_Angle(a_sim, w_sim, pipe_angle, geo);
       
    hold on   
    h = Cylinder(pipe_centre, pipe_centre2,r_p,40,'black',0,1,0.8,1);
      
%     x_imu_sim(i,:) = acc_x;
%     y_imu_sim(i,:) = acc_y;
%     z_imu_sim(i,:) = acc_z;

    xlim([-r_p-300 r_p+300])
    ylim([0-300 pipe_length+300])
    zlim([-r_p-300 r_p+300])
    daspect([1 1 1])
    view([1 -1 1])
    
    
    drawnow

end

% t_run_algo = mean(t)
% t_run_graphic = mean(t_g)
% t_opt_full = mean(t_opt)
% 
% % Plot Errors
% 
% a_errors = [abs(all_a(:,1)-all_a(:,2)) abs(all_a(:,1)-all_a(:,3))];
% 
% w_errors = [abs(all_w(:,1)-all_w(:,2)) abs(all_w(:,1)-all_w(:,3))];
% 
% figure
% subplot(2,1,1)
% erplt1 = plot(all_w(:,1),a_errors(:,1),all_w(:,1),a_errors(:,2));
% xlabel('Simulated Omega')
% ylabel('Alpha Error [°]')
% title('ABS Alpha Error')
% legend('Algorithm Error','Optimisation Error')
% 
% subplot(2,1,2)
% erplt2 = plot(all_w(:,1),w_errors(:,1),all_w(:,1),w_errors(:,2));
% xlabel('Simulated Omega')
% ylabel('Omega Error [°]')
% title('ABS Omega Error')
% legend('Algorithm Error','Optimisation Error')
% 
% set([erplt1, erplt2],'LineWidth',2)
% 
% 
% 
% %% Plots WRT OMEGA
% %line = linspace(0,1,length(A_sim));
% 
% % Angles
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
% %% Plots WRT ALPHA
% 
% % figure
% % subplot(2,1,1)
% % plot1 = plot(A_sim,all_a(:,1),'-o',A_sim,all_a(:,2),':s',A_sim, all_a(:,3),'-.*','markers',3);
% % title('Alpha vs. Omega')
% % legend('Simulated','Algorithm','Optimisation')
% % xlabel('Simulated Alpha [°]')
% % ylabel('Alphas [°]')
% % 
% % subplot(2,1,2)
% % plot2 = plot(A_sim,all_w(:,1),'-o',A_sim,all_w(:,2),':s',A_sim, all_w(:,3),'-.*','markers',3);
% % title('Omega vs. Omega')
% % legend('Simulated','Algorithm','Optimisation')
% % xlabel('Simulated Omega [°]')
% % ylabel('Omegas [°]') 
% 
% % set([plot1, plot2],'LineWidth',2)
% % % Accelerometer 
% % figure
% % subplot(3,1,1)
% % plot3 = plot(A_sim,x_imu_sim,'-o',A_sim,x_imu_algo,':s',A_sim, x_imu_opt,'-.*','markers',3);
% % title('X ACCEL')
% % legend('Simulated','Algorithm','Optimisation')
% % xlabel('Simulated Omega [°]')
% % ylabel('Acc [mm/s^2]')
% 
% % subplot(3,1,2)
% % plot4 = plot(A_sim,y_imu_sim,'-o',A_sim,y_imu_algo,':s',A_sim, y_imu_opt,'-.*','markers',3);
% % title('Y ACCEL')
% % legend('Simulated','Algorithm','Optimisation')
% % xlabel('Simulated Omega [°]')
% % ylabel('Acc [mm/s^2]')
% 
% % subplot(3,1,3)
% % plot5 = plot(A_sim,z_imu_sim,'-o',A_sim,z_imu_algo,':s',A_sim, z_imu_opt,'-.*','markers',3);
% % title('Z ACCEL')
% % legend('Simulated','Algorithm','Optimisation')
% % xlabel('Simulated Omega [°]')
% % ylabel('Acc [mm/s^2]')
% 
% % set([plot3, plot4, plot5],'LineWidth',2)
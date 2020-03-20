%% Plot VICON Data
% Plot the VICON data and the robot
% Fixes the orientation of the pipe and takes this into account when
% looking at the quaternions of the measured robot

close all
clear all

filename = 'omgworking.gif';

%% Variables
 b = 180; %fat
%b = 70; %tall
l = 180;
cylinder_length = 500;
r_p = 507/2;

% Centre of pipe, taken manually by fitting a circle to vicon points using pipe_cent.m script
% Big Triangle

xc = -2.7613+4; 
zc = 1.0157e+03+5;

% Long Triangle
% xc = 7.3228;
% zc = 1.0156e+03;

% Flat Correction for long triangle
%     x_corr = -0.053;
%     y_corr = 0.0246;
%     z_corr = -0.008;

x_corr = 0;
y_corr = 0;
z_corr = 0;

geo_know(1) = r_p;
geo_know(2) = b;
geo_know(3) = l;
geo_know(4) = cylinder_length; %sometimes this is changed to vicon_point(2)
geo_know(5) = xc;
geo_know(6) = zc;

geo_know(8)  =  x_corr;
geo_know(9)  =  y_corr;
geo_know(10) =  z_corr;

geo_know(11) = 22; % robot centre correction in x axis

% For plots
no_of_files = 10;
xAxOmega = linspace(1,360,no_of_files);
xAxAlpha = linspace(1,360,no_of_files);

for i = 1:no_of_files
figure
%    VICON Analysis Code
      vicon_file = sprintf('omega %d.csv',i);
%    vicon_file = sprintf('long_tri_omega %d.csv',i);
%    vicon_file = sprintf('long_tri_alpha %d.csv',i);
    
    vicon_data = csvread(vicon_file,5,2);
    vicon_data = mean(vicon_data(10:20,:));
    
    [accel_sim_vicon, x_axis, y_axis, z_axis, vicon_omega, vicon_alpha, vicon_point, side] = VICON_plot(vicon_data,geo_know)   ;
    
    geo_know(7) = side;
    
    vic_alpha(i) = vicon_alpha*180/pi;
    vic_omega(i) = vicon_omega*180/pi;
    
    % For Forward Model Paper
    
    accel_sim_vicon_list(i,:) = accel_sim_vicon;
    
    % IMU Analysis Code
       file_mti300 = sprintf('omega%d-000.csv',i); %
%     file_mti300 = sprintf('tall_tri_omega %d-000.csv',i);
%   file_mti300 = sprintf('tall_tri_alpha %d-000.csv',i);

    
    [opto_output_mti300,points, opto_omega_mti300, opto_alpha_mti300, algo_omega_mti300, algo_alpha_mti300, IMU_data] = IMU_algorithm(file_mti300, geo_know,[     0    0.4470    0.7410],vicon_point);
    
    imu_alpha_alg(i) = algo_alpha_mti300*180/pi;
    imu_omega_alg(i) = algo_omega_mti300*180/pi;    
    imu_alpha_opt(i) = opto_alpha_mti300*180/pi;
    imu_omega_opt(i) = opto_omega_mti300*180/pi;
    
    IMU_data_list(i,:) = IMU_data(1:3);
    
    Forward_model_diff(i,:) = abs(IMU_data_list(i,:) - accel_sim_vicon_list(i,:));
    
    Forward_model_percent_error(i,:) = abs(IMU_data_list(i,:) - accel_sim_vicon_list(i,:))./abs(IMU_data_list(i,:))*100;
    
    
    %% Parametric Study Comparison
    
xlim([-110 - 250 -110 + 250])
ylim([550 - 250 550 + 250])
zlim([250 - 250 250 + 250])

    
    set(gcf,'color','w');
    view([0 0 1])
    
    
      frame = getframe; 
      im = frame2im(frame); 
      [imind,cm] = rgb2ind(im,256); 
      % Write to the GIF File 
      if i == 1 
          imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
      else 
          imwrite(imind,cm,filename,'gif','WriteMode','append'); 
      end 
      
end

%% Just Vicon Plot
figure
fig_let = ["(A)","(B)","(C)","(D)","(E)","(F)"];
for i = 1:no_of_files
    
%    VICON Analysis Code
      vicon_file = sprintf('omega %d.csv',i);
%    vicon_file = sprintf('long_tri_omega %d.csv',i);
%    vicon_file = sprintf('long_tri_alpha %d.csv',i);
    
    vicon_data = csvread(vicon_file,5,2);
    vicon_data = mean(vicon_data(10:20,:));
    subplot(3,2,i)
    [accel_sim_vicon, x_axis, y_axis, z_axis, vicon_omega, vicon_alpha, vicon_point, side] = VICON_plot(vicon_data,geo_know);
    
    vicon_alpha = vicon_alpha-pi/2;
    vicon_omega = vicon_omega-pi/2;
    
    xlim([-110 - 250 -110 + 250])
    ylim([550 - 250 550 + 250])
    zlim([250 - 250 250 + 250])
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    set(gca,'XTick',[])
    set(gca,'YTick',[])
    set(gca,'ZTick',[])
    
    set(gcf,'color','w');
    view([0 0 1])
    fig_str = fig_let(i);
    title(sprintf(fig_str + ' \\alpha = %.1f°, \\omega = %.1f°', round(rad2deg(wrapTo2Pi(vicon_alpha)),1), round(rad2deg(wrapTo2Pi(vicon_omega)),1)))
    
end

figure
% Error: Dfference between Vicon Robot and Algo/Opto Output
eAlphaAlgo = (imu_alpha_alg - vic_alpha); 
eAlphaOpto = (imu_alpha_opt - vic_alpha); 

eOmegaAlgo = (imu_omega_alg - vic_omega); 
eOmegaOpto = (imu_omega_opt - vic_omega);

figure
    filename = 'errorgif.gif';
    set(gca,'color','w')
    
    subplot(2,1,1)
    an_omega_opt = animatedline('color','m','marker','x');
    an_omega_alg = animatedline('color','r','marker','o');
    
    title('\omega Error')
    xlabel('Vicon \alpha [°]')
    ylabel('\omega Error [°]')
    %ylim([min(eOmegaAlgo) max(eOmegaAlgo)])
    xlim([0 360])
    ylim([-20 20])
    legend('Optimisation','Algorithim')
    %legend('Algorithim')
    
    subplot(2,1,2)
    %ylim([min(eAlphaAlgo) max(eAlphaAlgo)])
    an_alpha_opt = animatedline('color','m','marker','x');
    an_alpha_alg = animatedline('color','r','marker','o');    
    ylim([-20 20])
    title('\alpha Error')
    xlabel('Vicon \alpha [°]')
    ylabel('\alpha Error [°]')
    xlim([0 360])
    ylim([-20 20])
    legend('Optimisation','Algorithm') 
     %legend('Algorithim')
    
for ii = 1:no_of_files
    
%     plot(xAxOmega(ii), eOmegaAlgo(ii),'-ro', 'LineWidth',2)
%     plot(xAxOmega(ii), eOmegaOpto(ii),'--mx', 'linewidth',2)

%     plot(xAxAlpha(ii), eAlphaAlgo(ii),'-ro', 'linewidth',2)
%     plot(xAxAlpha(ii), eAlphaOpto(ii),'--mx', 'linewidth',2)

%     subplot(2,1,2)
%     addpoints(an_omega_opt, xAxOmega(ii), eOmegaOpto(ii));
%     addpoints(an_omega_alg, xAxOmega(ii), eOmegaAlgo(ii));
%     subplot(2,1,2)
%     addpoints(an_alpha_opt, xAxAlpha(ii), eAlphaOpto(ii));
%     addpoints(an_alpha_alg, xAxAlpha(ii), eAlphaAlgo(ii));
% 
%         drawnow
%         frame = getframe(figure(17)); 
%         im = frame2im(frame); 
%         [imind,cm] = rgb2ind(im,256); 
%         %Write to the GIF File 
%         if ii == 1 
%             imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%         else 
%             imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%         end 
    
end

zero_lin = linspace(1,360,360);
zero_lin2 = zeros(length(zero_lin),1);

figure

subplot(2,1,1)
%an_omega = animatedline('color','m','marker','x');
title('\omega Error')
xlabel('Vicon \alpha [°]')
ylabel('\omega Error [°]')
ylim([min(eOmegaAlgo) max(eOmegaAlgo)])
xlim([0 360])

hold on

plot(xAxOmega, eOmegaAlgo,'-ro', 'LineWidth',2)
plot(xAxOmega, eOmegaOpto,'--mx', 'linewidth',2)
plot(zero_lin, zero_lin2,'--k','linewidth',2);
ylim([-20 20])
drawnow

legend('Algorithim','Optimisation')
hold off

subplot(2,1,2)
%an_alpha_opt = animatedline('color','m','marker','x');

title('\alpha Error')
xlabel('Vicon \alpha [°]')
ylabel('\alpha Error [°]')
xlim([0 360])
hold on

plot(xAxAlpha, eAlphaAlgo,'-ro', 'linewidth',2)
plot(xAxAlpha, eAlphaOpto,'--mx', 'linewidth',2)
plot(zero_lin, zero_lin2,'--k','linewidth',2);
ylim([-20 20])
legend('Algorithm','Optimisation')

hold off

 filename = 'errorgif.gif';
 set(gca,'color','w')
 
%  
% for j = 1:no_of_files
%    subplot(2,1,1)
%    addpoints(an_omega, xAxOmega(j), eOmegaOpto(j));
%    subplot(2,1,2)
%    addpoints(an_alpha, xAxAlpha(j), eAlphaOpto(j));
%     drawnow
%     frame = getframe(figure(20)); 
%       im = frame2im(frame); 
%       [imind,cm] = rgb2ind(im,256); 
%       % Write to the GIF File 
%       if j == 1 
%           imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%       else 
%           imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%       end 
% end
% 
% 
% for j = 1:15
%     subplot(2,1,1)
%     plot(xAxOmega, eOmegaAlgo,'r', 'LineWidth',2,'Marker','o')
%     
%        subplot(2,1,2)
%     plot(xAxAlpha, eAlphaAlgo,'r', 'linewidth',2,'Marker','o')
%             
%     frame = getframe(gca); 
%       im = frame2im(frame); 
%       [imind,cm] = rgb2ind(im,256); 
%       % Write to the GIF File 
%       if j == 1 
%           imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%       else 
%           imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%       end 
%      
%     
% end


Forward_model_diff
Forward_model_percent_error

mean(Forward_model_diff)
range(Forward_model_diff)
std(Forward_model_diff)

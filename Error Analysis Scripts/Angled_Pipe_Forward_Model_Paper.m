%% Angled Pipes Forward Model


%% 
close all
clear all
filename = '0_deg_opt.gif';
folder = '/GIFS';

%% Parameters
pipe_length = 500;
r_p             = 250;
pipe_angle = 0*pi/180;
b   = 100;
l   = 125; 
Y_p = 0.5*pipe_length;
do_plot = 1;

%% Pipe 

pipe_centre = [0  0 0];
pipe_centre2 = [0 pipe_length*cos(pipe_angle) pipe_length*sin(pipe_angle)];

%% Angles
W_sim = linspace(1,359,50);
%A_sim = linspace(1,359,100);
A_sim(1:length(W_sim))  = 45;
%W_sim(1:length(A_sim)) = 45;

fig1 = figure;
set(fig1, 'Position', [100, 100, 1000, 900])


geo(1) = r_p;
geo(2) = b; 
geo(3) = l;
geo(4) = Y_p;
    
% Rotation Matricies for w 
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

    % Forward Model
    
    [wheels, acc] = Forward_Model_Angle(a_sim, w_sim, pipe_angle, geo, do_plot);
       
    wheel_1_sim = wheels(1,:);
    wheel_2_sim = wheels(2,:);
    castor_sim = wheels(3,:);
    
    acc_x = acc(1);
    acc_y = acc(2);
    acc_z = acc(3);
    
    % Inverse Model
    
    
    
    % Optimisation
    
    hold on   
    h = Cylinder(pipe_centre, pipe_centre2,r_p,40,'black',0,1,0.8,1);
      

    xlim([-r_p-300 r_p+300])
    ylim([0-300 pipe_length+300])
    zlim([-r_p-300 r_p+300])
    daspect([1 1 1])
    view([1 -1 1])
    
    drawnow

end

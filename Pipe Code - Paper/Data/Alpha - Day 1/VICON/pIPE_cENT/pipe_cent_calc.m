%% Calculate Pipe Centre from VICON data
quat_p = [0.9999   -0.0000   -0.0001    0.0105];

for i = 1:4
    
    %vicon_data = sprintf('long_tri_omega %d.csv',i);
    vicon_data = sprintf('pipe_cent %d.csv',i);
    
    vicon_data = csvread(vicon_data,5,2);
    vicon_data = mean(vicon_data(10:20,:));
    points(i,:) = [vicon_data(5) vicon_data(6) vicon_data(7)];
   
    vicon_point_straight = quatrotate(quat_p,points(i,:));
    points_straight(i,:) =  points(i,:);
    
    
end





figure
scatter3(points_straight(:,1),points_straight(:,2),points_straight(:,3))
[xc,zc, R] = circfit(points_straight(:,1), points_straight(:,3))
hold on
scatter3(xc,0,zc,'r')
axis equal


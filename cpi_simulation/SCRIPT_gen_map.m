%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% generate the map for stereo vision in one direction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clear all
close all
clc

lm_sim_X = [];

%% generate points within a cylinder r=500mm, height=2000mm. 
% lm_num = 4000;
% r = 8000;
% dr = 100* rand(1,lm_num);
% th = 2*pi*rand(1,lm_num);
% lm_x= (r+dr).*cos(th);
% lm_z= linspace(-4000,15000,lm_num);
% lm_y =(r+dr).*sin(th);
% lm_sim_X(1,:) = lm_x;
% lm_sim_X(2,:) = lm_y;
% lm_sim_X(3,:) = lm_z;


% r = 18000;
% ringheight = 800;
% n_per_round = 80;
% th_per_round = linspace(0,2 * pi, n_per_round);
% x_per_round = r * sin(th_per_round);
% y_per_round = r * cos(th_per_round);
% for i=0:ringheight:10000
%     z_per_round = ones(1,n_per_round) * (i);
%     lm_per_round = [x_per_round; y_per_round; z_per_round];
%     lm_sim_X = [lm_sim_X lm_per_round];
% end
% lm_sim_X = lm_sim_X./1000;


%% generate regular straight line points
% r = 4000;%9000;
% height = 1000;
% n_per_round = 80;
% th_per_round = linspace(0,2 * pi, n_per_round);
% x_per_round = r * sin(th_per_round);
% y_per_round = r * cos(th_per_round);
% lm_sim_X = [];
% 
% for i=-20000:height:320000
%     z_per_round = ones(1,n_per_round) * (i);
%     lm_per_round = [x_per_round; z_per_round; y_per_round];
%     lm_sim_X = [lm_sim_X lm_per_round];
% end
% lm_num = size(lm_sim_X,2);
% lm_sim_X = lm_sim_X./1000;



%% Static flat "floor" points

height = 0;
spacing = 1;
limits = 20;

% for i = -limits:spacing:limits
%     for j = -limits:spacing:limits
%         nfeats = [i; j; height];
%         lm_sim_X = [lm_sim_X nfeats];
%     end
% end

for height = 6:spacing:15
    for i = -limits:spacing:limits
        for j = -limits:spacing:limits
            nfeats = [i; j; height];
            lm_sim_X = [lm_sim_X nfeats];
        end
    end
end

%% Plot the feature map for inspection


figure('name','The map for line motion')
plot3( lm_sim_X(1,:),lm_sim_X(2,:),lm_sim_X(3,:),'r+');
grid on
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on,
hold on
% for i=1:lm_num
%     %text(lm_sim_X(1,i)+20,lm_sim_X(2,i)+20,lm_sim_X(3,i)+20,num2str(lm_sim_X(4,i)));
% end
hold on
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Map');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%trajectory for the camera
% dt = 0.5;
% Tr_N = 840;
% Tr_w = 0.03;
% Tr_r_base = 1250;
% Tr_h = 1000;
% Tr_camera = zeros(3,Tr_N);
% 
% Tr_theta = dt*Tr_w*(0:(Tr_N-1));
% Tr_r = Tr_r_base*ones(1,Tr_N);
% Tr_camera(1,:) = Tr_r.*cos(Tr_theta);
% Tr_camera(2,:) = Tr_r.*sin(Tr_theta);
% Tr_camera(3,:) = Tr_h/(2*pi)*Tr_theta;
% 
% plot3(Tr_camera(1,:),Tr_camera(2,:),Tr_camera(3,:),'->');
% 
% xlabel('X');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% grid on,
% hold off

% ylabel('Y');
% zlabel('Z');
% grid on,
% hold off



% randomize the ordering of theses features
lm_sim_X = lm_sim_X(:,randperm(size(lm_sim_X, 2)));

% store the feature in our map object
map.feats=lm_sim_X(1:3,:);
map.nfeats=120;
map.nfeats_valid=length(map.feats);
map.nfeats_per_im=120;
save data_maps/map_02.mat map;




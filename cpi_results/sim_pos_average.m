% MIT License
% 
% Copyright (c) 2018 Kevin Eckenhoff
% Copyright (c) 2018 Patrick Geneva
% Copyright (c) 2018 Guoquan Huang
% 
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
% 
% The above copyright notice and this permission notice shall be included in all
% copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
% SOFTWARE.

%% Generates 3D position plots for the pose files
% Settings

% Add libraries
addpath('functions')
addpath('functions/dist_colors')

% Clear old variables
clear all

% Font size on the plots
fontsize = 20;
linewidth = 1.5;

% Save to file (set to 1 to save)
save_to_file = 0;

% If we should plot the simulated features
plot_features = 0;

%% Load our data and settings from file


load_data;




%% Unique Timestamps (simulation)
% Get the unique sets in comparision to the first one
fprintf('Finding common timestamps...\n') 
times = data_e{end}.data(:,1);
for ii=1:length(data_e)-1
    % get the intersection
    [times,ia,ib] = intersect(times,data_e{ii}.data(:,1));
    % debug printing
    %fprintf('%d => %d common timestamps found!\n',ii,size(times,1))  
end
fprintf('%d common timestamps found!\n',size(times,1)) 

% Convert them to all have the same!
for ii=1:length(data_e)
    % get the intersection
    [~,ia,ib] = intersect(times,data_e{ii}.data(:,1));
    % debug printing
    %fprintf('%d => %d timestamps converted!\n',size(data_e{ii}.data,1),size(ia,1))
    % Save it
    data_e{ii}.data = data_e{ii}.data(ib,:);
end

%% Find the averages

% Subtract out the first timestamp
% This will give us relative time to the first recorded pose
timestamps = bsxfun(@minus,data_e{1}.data(:,1),data_e{1}.data(1,1));

% Loop through and calculate position average
traj_avg = {};
for i=0:length(data_e)/numruns-1
    traj_avg{i+1} = zeros(length(timestamps),3);
    fprintf('New array for %s\n',data_n{i+1})
    for j=1:numruns
        traj_avg{i+1}(:,1) = traj_avg{i+1}(:,1) + data_e{i*numruns+j}.data(:,2);
        traj_avg{i+1}(:,2) = traj_avg{i+1}(:,2) + data_e{i*numruns+j}.data(:,3);
        traj_avg{i+1}(:,3) = traj_avg{i+1}(:,3) + data_e{i*numruns+j}.data(:,4);
        fprintf('   + summing at index %d from original index %d\n',i+1,i*numruns+j)
    end
end

% average by the number of runs
for i=1:length(traj_avg)
    traj_avg{i}(:,1) = bsxfun(@rdivide,traj_avg{i}(:,1),numruns);
    traj_avg{i}(:,2) = bsxfun(@rdivide,traj_avg{i}(:,2),numruns);
    traj_avg{i}(:,3) = bsxfun(@rdivide,traj_avg{i}(:,3),numruns);
end


%% Loop through and calculate distance
dist = zeros(length(traj_avg)+1,1);
vel = zeros(length(traj_avg)+1,1);
for i=1:length(traj_avg)
for index = 1:stop_num-1
    temp = (traj_avg{i}(index,1) - traj_avg{i}(index+1,1))^2 ...
            + (traj_avg{i}(index,2) - traj_avg{i}(index+1,2))^2 ...
            + (traj_avg{i}(index,3) - traj_avg{i}(index+1,3))^2;
     dist(i) = dist(i) + sqrt(temp);
     vel(i) = vel(i) + norm(traj_avg{i}(index+1,1:3)-traj_avg{i}(index,1:3))./(timestamps(index+1)-timestamps(index));
end
end
% Ground truth calculation
for index = 1:stop_num-1
    temp = (data_e{end}.data(index,2) - data_e{end}.data(index+1,2))^2 ...
            + (data_e{end}.data(index,3) - data_e{end}.data(index+1,3))^2 ...
            + (data_e{end}.data(index,4) - data_e{end}.data(index+1,4))^2;
     dist(end) = dist(end) + sqrt(temp);
     vel(end) = vel(end) + norm(data_e{end}.data(index+1,2:4)-data_e{end}.data(index,2:4))./(timestamps(index+1)-timestamps(index));
end

% average velocity
vel = vel./(stop_num-1);

% Print the distance traveled out
for i=1:length(dist)
	fprintf('Disance Traveled = %.4f (m) | velocity = %.4f (m/s) [%s]\n',dist(i),vel(i),data_n{i});
end


%% Plot the graphs

% Close all old plots
close all

fh1 = figure(1);
set(gcf,'PaperPositionMode','auto')
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultuicontrolfontname','Bitstream Charter');
set(gcf,'DefaultAxesFontSize',fontsize);
set(gcf,'DefaultAxesFontName','Bitstream Charter');
set(gcf,'DefaultTextFontSize',fontsize);
set(gcf,'DefaultTextFontname','Bitstream Charter');
plot3(data_e{end}.data(1:skip_num:stop_num,2),data_e{end}.data(1:skip_num:stop_num,3),data_e{end}.data(1:skip_num:stop_num,4),'-b','LineWidth',linewidth); hold on;
grid on
xlabel('x-distance (m)');
ylabel('y-distance (m)');
zlabel('z-distance (m)');
xlim([min(data_e{end}.data(1:skip_num:stop_num,2)) max(data_e{end}.data(1:skip_num:stop_num,2))])
ylim([min(data_e{end}.data(1:skip_num:stop_num,3)) max(data_e{end}.data(1:skip_num:stop_num,3))])
zlim([min(data_e{end}.data(1:skip_num:stop_num,4)) max(data_e{end}.data(1:skip_num:stop_num,4))])
view([32 50]);
set(gcf,'Position',[0 0 1000 600])
set(get(gca,'xlabel'),'rotation',-12)
set(get(gca,'ylabel'),'rotation',36)

plot3(data_e{end}.data(1,2),data_e{end}.data(1,3),data_e{end}.data(1,4),'gs', 'MarkerSize',20,'MarkerFaceColor','g'); hold on;
plot3(data_e{end}.data(stop_num,2),data_e{end}.data(stop_num,3),data_e{end}.data(stop_num,4),'rd','MarkerSize',20,'MarkerFaceColor','r'); hold on;


%====================================================================
% plot just the ground truth
fh2 = figure(2);
set(gcf,'PaperPositionMode','auto')
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultuicontrolfontname','Bitstream Charter');
set(gcf,'DefaultAxesFontSize',fontsize);
set(gcf,'DefaultAxesFontName','Bitstream Charter');
set(gcf,'DefaultTextFontSize',fontsize);
set(gcf,'DefaultTextFontname','Bitstream Charter');
% Plot each 3d position
for i=1:length(traj_avg)
    plot3(traj_avg{i}(1:skip_num:stop_num,1),traj_avg{i}(1:skip_num:stop_num,2),traj_avg{i}(1:skip_num:stop_num,3),data_d{i},'Color',data_c(i,:),'LineWidth',linewidth); hold on;
end
% Plot groundtruth
plot3(data_e{end}.data(1:skip_num:stop_num,2),data_e{end}.data(1:skip_num:stop_num,3),data_e{end}.data(1:skip_num:stop_num,4),'-b','LineWidth',linewidth); hold on;
grid on
xlabel('x-distance (m)');
ylabel('y-distance (m)');
zlabel('z-distance (m)');
% xlim([min(data_e{end}.data(1:skip_num:stop_num,2)) max(data_e{end}.data(1:skip_num:stop_num,2))])
% ylim([min(data_e{end}.data(1:skip_num:stop_num,3)) max(data_e{end}.data(1:skip_num:stop_num,3))])
% zlim([min(data_e{end}.data(1:skip_num:stop_num,4)) max(data_e{end}.data(1:skip_num:stop_num,4))])

%axis equal
%view([90 90])
legend(data_n,'Location','southeast');

view([32 50]);
set(gcf,'Position',[0 0 1000 600])
set(get(gca,'xlabel'),'rotation',-12)
set(get(gca,'ylabel'),'rotation',36)


%% Save to file
if save_to_file
    print(fh1,'-dpng','-r500',[path_main,'plot_3d_path.png'])
    print(fh2,'-dpng','-r500',[path_main,'plot_3d_pathgt.png'])
end

%% PLOT FEATURES IF WE HAVE THEM
if plot_features
    % load features and plot them!
    load([path_main,'data_gt/map_01.mat'])
    plot3(map.feats(1,:),map.feats(2,:),map.feats(3,:),'.'); hold on;
    % save this to file if needed
    if save_to_file
        print(fh1,'-dpng','-r500',[path_main,'plot_3d_path_features.png'])
    end
end



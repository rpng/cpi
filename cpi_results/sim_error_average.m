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


%% Generates error for the system in respect to some truth
% Settings

% Add libraries
addpath('functions')
addpath('functions/dist_colors')

% Close all old plots
close all
clear all

% Font size on the plots
fontsize = 16;
linewidth = 1;

% Save to file (set to 1 to save)
save_to_file = 0;


%% Load our data and settings from file


load_data;




%% Next interpolate all data based on the toto output (time_cg, pos_g, time_e)
% Get the unique sets in comparision to the first one
fprintf('Finding common timestamps...\n') 
times = data_e{end}.data(:,1);
for ii=1:length(data_e)-1
    % get the intersection
    [times,ia,ib] = intersect(times,data_e{ii}.data(:,1));
    % debug printing
    fprintf('%d => %d common timestamps found!\n',ii,size(times,1))  
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



%% Calculate the error between each system in respect to ground truth

% Subtract out the first timestamp
% This will give us relative time to the first recorded pose
timestamps = bsxfun(@minus,data_e{1}.data(:,1),data_e{1}.data(1,1));

% Calculate the difference between the ground truth and estimates
% Note it is in respect to the GPS which is in the last cell
diff_e = cell(1,length(data_e)-1);
for i=1:length(data_e)-1
    % Position difference
    diff_e{i} = abs(data_e{end}.data(:,2:4)-data_e{i}.data(:,2:4));
    % Angle difference
    for jj=1:size(data_e{end}.data,1)
        quatdiff = quat_mul(data_e{i}.data(jj,5:8)',quat_inv(data_e{end}.data(jj,5:8)'));
        diff_e{i}(jj,4) = norm(2.*quatdiff(1:3));
    end
end



%% Find the average of all the errors
diff_avg = {};
for i=0:length(diff_e)/numruns-1
    diff_avg{i+1} = zeros(length(timestamps),4);
    fprintf('New array for %s\n',data_n{i+1})
    for j=1:numruns
        diff_avg{i+1}(:,1) = diff_avg{i+1}(:,1) + abs(diff_e{i*numruns+j}(:,1));
        diff_avg{i+1}(:,2) = diff_avg{i+1}(:,2) + abs(diff_e{i*numruns+j}(:,2));
        diff_avg{i+1}(:,3) = diff_avg{i+1}(:,3) + abs(diff_e{i*numruns+j}(:,3));
        diff_avg{i+1}(:,4) = diff_avg{i+1}(:,4) + abs(diff_e{i*numruns+j}(:,4));
        fprintf('   + summing at index %d from original index %d\n',i+1,i*numruns+j)
    end
end

% average by the number of runs
for i=1:length(diff_avg)
    diff_avg{i}(:,1) = bsxfun(@rdivide,diff_avg{i}(:,1),numruns);
    diff_avg{i}(:,2) = bsxfun(@rdivide,diff_avg{i}(:,2),numruns);
    diff_avg{i}(:,3) = bsxfun(@rdivide,diff_avg{i}(:,3),numruns);
    diff_avg{i}(:,4) = bsxfun(@rdivide,diff_avg{i}(:,4),numruns);
end

% Calculate the square norm of the average error
% Note it is in respect to the GPS which is in the last cell
diff_en = {};
for i=1:length(diff_avg)
    diff_en{i} = diff_avg{i}(:,1).^2 + diff_avg{i}(:,2).^2 + diff_avg{i}(:,3).^2;
    diff_en{i} = sqrt(diff_en{i});
end

%% Print it out
fprintf('number of runs = %d \n',numruns);
fprintf('number of types = %d \n',length(diff_en));

for i=1:length(diff_en)
    fprintf('average average error (m) = %.5f | (deg) = %.5f [%s]\n',mean(diff_en{i}(1:skip_num:stop_num)),180/pi.*mean(diff_avg{i}(1:skip_num:stop_num,4)),data_n{i});
end


%% Plot the graphs
% 3-AXIS ERROR PLOT
fh1 = figure('name','Position Errors');
set(gcf,'PaperPositionMode','auto')
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultuicontrolfontname','Bitstream Charter');
set(gcf,'DefaultAxesFontSize',fontsize);
set(gcf,'DefaultAxesFontName','Bitstream Charter');
set(gcf,'DefaultTextFontSize',fontsize);
set(gcf,'DefaultTextFontname','Bitstream Charter');
yNames = {'x-pos error','y-pos error','z-pos error'};

for i = 1:3
    % Specify our current plot
    subplot(3,1,i);
    % Loop through each diff
    for j=1:length(diff_avg)
        plot(timestamps(1:skip_num:stop_num),diff_avg{j}(1:skip_num:stop_num,i),data_d{j},'Color',data_c(j,:),'LineWidth',linewidth); hold on;
    end
    % Axis label and limit
    ylabel(yNames{i});
    xlim([min(timestamps(1:skip_num:stop_num)) max(timestamps(1:skip_num:stop_num))])
    %ylim([0 0.3])
    % Sigma bound of our first estimate
    %plot(timestamps(1:skip_num:stop_num),3*sqrt(data_e{1}.data(1:skip_num:stop_num,7+i))','r');
end
xlabel('Time (sec)');
xlim([min(timestamps(1:skip_num:stop_num)) max(timestamps(1:skip_num:stop_num))])
legend(data_n(1:end-1));
set(gcf,'Position',[0 0 1400 600])


% NORM ERROR PLOT POSITION
fh2 = figure('name','Norm Error Position');
set(gcf,'PaperPositionMode','auto');
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultuicontrolfontname','Bitstream Charter');
set(gcf,'DefaultAxesFontSize',fontsize);
set(gcf,'DefaultAxesFontName','Bitstream Charter');
set(gcf,'DefaultTextFontSize',fontsize);
set(gcf,'DefaultTextFontname','Bitstream Charter');
for j=1:length(diff_en)
    plot(timestamps(1:skip_num:stop_num),diff_en{j}(1:skip_num:stop_num),data_d{j},'Color',data_c(j,:),'LineWidth',linewidth); hold on;
end
ylabel('Error (m)');
%xlabel('Time (sec)');
xlim([min(timestamps(1:skip_num:stop_num)) max(timestamps(1:skip_num:stop_num))])
%ylim([0 2])
legend(data_n(1:end-1),'Location','BestOutside');
%legend(data_n(1:end-1),'Location','SouthEast');
set(gcf,'Position',[0 0 1000 220])


% % NORM ERROR PLOT ORIENTATION
fh3 = figure('name','Norm Error Orientation');
set(gcf,'PaperPositionMode','auto');
set(gcf,'defaultuicontrolfontsize',fontsize);
set(gcf,'defaultuicontrolfontname','Bitstream Charter');
set(gcf,'DefaultAxesFontSize',fontsize);
set(gcf,'DefaultAxesFontName','Bitstream Charter');
set(gcf,'DefaultTextFontSize',fontsize);
set(gcf,'DefaultTextFontname','Bitstream Charter');
for j=1:length(diff_avg)
    plot(timestamps(1:skip_num:stop_num),180/pi.*diff_avg{j}(1:skip_num:stop_num,4),data_d{j},'Color',data_c(j,:),'LineWidth',linewidth); hold on;
end
ylabel('Error (deg)');
xlabel('Time (sec)');
xlim([min(timestamps(1:skip_num:stop_num)) max(timestamps(1:skip_num:stop_num))])
%ylim([0 1])
legend(data_n(1:end-1),'Location','BestOutside');
%legend(data_n(1:end-1),'Location','SouthEast');
set(gcf,'Position',[0 0 1000 250])



%% Save them to file
if save_to_file
    %print(fh1,'-dpng','-r500',[path_main,'error_avg_3axis.png'])
    print(fh2,'-dpng','-r900',[path_main,'normpos.png'])
    print(fh3,'-dpng','-r900',[path_main,'normang.png'])
end





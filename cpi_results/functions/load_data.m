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

%% READ IN OUR DATA!!

% Number of datapoints to plot
delimiterIn = ' ';
skip_num = 1;
headerlinesIn = 1;

% Define our array of pose data
data_e = {};
data_d = {'-','-','-','-','-','-','-','-','-','-','-','-','-','-','-','-'};
data_c = distinguishable_colors(10, {'w','y'});
data_c(6,:) = data_c(10,:);

% Import the files (note that RTKGPS should always be last)
path_main = './';
path_sub = 'data_est/';
path_gt = 'data_gt/';

stop_num = 430;
%stop_num = 250;
data_n = {'MODEL-1','MODEL-2','DISCRETE','GT'};


% debug info
disp('Reading files from disk....')

% CPI MODEL v1
disp('CPI v1')
numruns = 0;
for file = dir([path_main,path_sub,'*.txt'])'
    filesplit = strsplit(file.name,'_');
    if any(strcmp(filesplit,'cpi1.txt'))
        disp(file.name)
        data_e{length(data_e)+1} = importdata([path_main,path_sub,file.name],delimiterIn,headerlinesIn);
        numruns = numruns + 1;
    end
end

% CPI MODEL v2
disp('CPI v2')
for file = dir([path_main,path_sub,'*.txt'])'
    filesplit = strsplit(file.name,'_');
    if any(strcmp(filesplit,'cpi2.txt'))
        disp(file.name)
        data_e{length(data_e)+1} = importdata([path_main,path_sub,file.name],delimiterIn,headerlinesIn);
    end
end

% FORSTER
disp('FORSTER')
for file = dir([path_main,path_sub,'*.txt'])'
    filesplit = strsplit(file.name,'_');
    if any(strcmp(filesplit,'forster.txt'))
        disp(file.name)
        data_e{length(data_e)+1} = importdata([path_main,path_sub,file.name],delimiterIn,headerlinesIn);
    end
end



% ===================================================================
% GROUNDTRUTH TRAJECTORY
% ===================================================================
data_e{length(data_e)+1} = importdata([path_main,path_gt,'groundtruth_imu_100hz.dat'],' ',headerlinesIn);
temp(:,1) =  1e-3.*data_e{length(data_e)}.data(:,9);
temp(:,2:4) = data_e{length(data_e)}.data(:,5:7);
temp(:,5:8) = data_e{length(data_e)}.data(:,1:4);
[~,idx] = sort(temp(:,1));
data_e{length(data_e)}.data = temp(idx,:);



% Remove dupes
for i=1:length(data_e)
    [~, index] = unique(data_e{i}.data(:,1));
    data_e{i}.data = data_e{i}.data(index,:);
end








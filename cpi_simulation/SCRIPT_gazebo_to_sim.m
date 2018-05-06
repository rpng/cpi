%% CONFIGURATION

addpath('robotics3D')

%close all;
%clear all;

bool_show_map_waypoints = 0;

%======================================================================================================
%======================================================================================================
% 100hz
path_imugazebo = './data_gazebo/100hz_circle-firefly-ground_truth-imu.csv';
path_posegazebo = './data_gazebo/100hz_circle-firefly-ground_truth-pose_with_covariance.csv';

% 200hz
%path_imugazebo = './data_gazebo/200hz_circle-firefly-ground_truth-imu.csv';
%path_posegazebo = './data_gazebo/200hz_circle-firefly-ground_truth-pose_with_covariance.csv';

% 400hz
%path_imugazebo = './data_gazebo/400hz_circle-firefly-ground_truth-imu.csv';
%path_posegazebo = './data_gazebo/400hz_circle-firefly-ground_truth-pose_with_covariance.csv';

% 800hz
%path_imugazebo = './data_gazebo/800hz_circle-firefly-ground_truth-imu.csv';
%path_posegazebo = './data_gazebo/800hz_circle-firefly-ground_truth-pose_with_covariance.csv';


%======================================================================================================
%======================================================================================================


% frequency of sensors (used in noise and camera measurement creation)
rate_sim = 100; %rate our IMU sim file was recorded at
rate_imu = 100; %rate we want to publish IMU messages
rate_cam = 10;
fprintf('simulating %d hz imu  with %d hz camera\n',rate_imu,rate_cam);

% Continous time noise variables
% These ones are modeled after the ADIS16448 sensor
gyroscope_noise_density = 0.005; %rad/s/sqrt(hz)
accelerometer_noise_density = 0.01; %m/s^2/sqrt(hz)
gyroscope_random_walk = 4.0e-06; %rad/s^2/sqrt(hz)
accelerometer_random_walk = 0.0002; %m/s^3/sqrt(hz)

% These ones are modeled after the ETH EurocMav dataset
% NOTE: graph solver solves when gyronoise density inflated by 1 power
% gyroscope_noise_density = 1.6968e-04; %rad/s/sqrt(hz)
% accelerometer_noise_density = 2.0000e-3; %m/s^2/sqrt(hz)
% gyroscope_random_walk = 1.9393e-05; %rad/s^2/sqrt(hz)
% accelerometer_random_walk = 3.0000e-3; %m/s^3/sqrt(hz)

% Our UV coordinate sigma
pixelsigma = 1/484.13161; %normalized px

% Extrinsic transforms between IMU to the CAMERAs
i_P_camera = zeros(3,1);
i_R_camera = eye(3); % no difference between camera imu frame
%i_R_camera = [0,0,-1; 0,1,0; 1,0,0]; %negative 90 about y-axis
IMU2Cam.q = rot2quat(i_R_camera); %left camera
IMU2Cam.p = i_P_camera; %left camera
IMU2Cam2.q = rot2quat(i_R_camera); %right camera
IMU2Cam2.p = i_P_camera + [0.1 0 0]'; %right camera



%% LOAD MAP AND DATA FROM FILE

fprintf('Reading map file ...\n');
load data_maps/map_01.mat;


fprintf('Reading gazebo groundtruth ...\n');
% Read in the groundtruth data that we generated in gazebo
% We converted this to a text file using the following repository:
% https://github.com/AtsushiSakai/rosbag_to_csv
data_true_IMU = readtable(path_imugazebo,'Delimiter',',','Format','%q%f%f%f%q%f%f%f%f%q%f%f%f%q%f%f%f%q');
data_true_POSE = readtable(path_posegazebo,'Delimiter',',','Format','%q%f%f%f%q%f%f%f%f%f%f%f%q');

% Debug print
fprintf('BEFORE: %d IMU readings and %d POSE readings...\n',size(data_true_IMU,1),size(data_true_POSE,1))

% only use matching timestamps
timesimu = table2array(data_true_IMU(:,3)) + 1e-9*table2array(data_true_IMU(:,4));
timespose = table2array(data_true_POSE(:,3)) + 1e-9*table2array(data_true_POSE(:,4));
[~,ia,ib] = intersect(timesimu,timespose);
data_true_IMU = data_true_IMU(ia,:);
data_true_POSE = data_true_POSE(ib,:);

% Debug print
fprintf('AFTER: %d IMU readings and %d POSE readings...\n',size(data_true_IMU,1),size(data_true_POSE,1))

% verify that our two data are of the same size
% each pose entry should match the IMU entry time
if size(data_true_IMU,1) ~= size(data_true_POSE,1)
   error('Data from gazebo has size mismatch, we need to have equal poses and IMU measurements (%d vs %d)!',size(data_true_IMU,1),size(data_true_POSE,1))
end


%% RANDOM SEED, FOR OUR MEASUREMENTS
rng('shuffle')

% initalize our bias value to zero
biasg = zeros(1,3);
biasa = zeros(1,3);

% setup our final data output
data.imu_true = [];
data.imu_meas = [];
data.pose_true_imu = [];
data.pose_true_gt = [];

% ID of what image we are on
ImgInd = 1;
ImuInd = 1;

% loop through each imu/pose measurement
for ii=1:size(data_true_IMU,1)-1
    
    % Nice debug printing
    if mod(ii,10*rate_sim)==0
        fprintf('Time Step: %i (%2.0f%%) \n',ii,ii/size(data_true_IMU,1)*100);
    end
    
    % skip this IMU message based on our sim rate
    % freq_imu/freq_sim = (1/rateimu)/(1/ratesim) = ratesim/rateimu
    if mod(ii,floor(rate_sim/rate_imu)) == 0
    
        % debug
        %fprintf('IMU = %d at %d\n',ImuInd,ii);

        % IMU delta time (we should use the actual timestamps to get accurate dt)
        timecurr = table2array(data_true_IMU(ii,3)) + 1e-9*table2array(data_true_IMU(ii,4));
        timenext = table2array(data_true_IMU(ii+floor(rate_sim/rate_imu),3)) + 1e-9*table2array(data_true_IMU(ii+floor(rate_sim/rate_imu),4));
        deltat = timenext-timecurr;

        % Our true IMU measurement
        data.imu_true(ImuInd,1:3) = table2array(data_true_IMU(ii,11:13)); %angular velocity
        data.imu_true(ImuInd,4:6) = table2array(data_true_IMU(ii,15:17)); %linear acceleration
        data.imu_true(ImuInd,8) = table2array(data_true_IMU(ii,3)) + 1e-9*table2array(data_true_IMU(ii,4)); % time in seconds

        % Add noise to each measurement direction
        data.imu_meas(ImuInd,1:3) = data.imu_true(ImuInd,1:3) + biasg + gyroscope_noise_density.*1./sqrt(deltat).*randn(1,3); %angular velocity
        data.imu_meas(ImuInd,4:6) = data.imu_true(ImuInd,4:6) + biasa + accelerometer_noise_density.*1./sqrt(deltat).*randn(1,3); %linear acceleration
        data.imu_meas(ImuInd,8) = data.imu_true(ImuInd,8); % time in seconds

        % Move the biases forward in time
        biasg = biasg + gyroscope_random_walk.*sqrt(deltat).*randn(1,3);
        biasa = biasa + accelerometer_random_walk.*sqrt(deltat).*randn(1,3);

        % Our true pose of the IMU
        data.pose_true_imu(ImuInd,1:4) = table2array(data_true_POSE(ii,9:12)); %orientation
        data.pose_true_imu(ImuInd,5:7) = table2array(data_true_POSE(ii,6:8)); %position
        data.pose_true_imu(ImuInd,9) = data.imu_true(ImuInd,8); % time in seconds

        % 4th element is always positive
        if data.pose_true_imu(ImuInd,4) < 0
            data.pose_true_imu(ImuInd,1:4) = -data.pose_true_imu(ImuInd,1:4);
        end

        % move IMU timestep forward
        ImuInd = ImuInd + 1;
    
    end
    
    % Should we publish a camera measurement?
    % freq_cam/freq_sim = (1/ratecam)/(1/ratesim) = ratesim/ratecam
    if mod(ii,floor(rate_sim/rate_cam)) == 0
        
        % debug
        %fprintf('CAM = %d at %d\n',ImgInd,ii);
        
        % we want to use the same functions, so for now create temp
        % variables to pass to the old simulator functions
        xtk(1:4,1) = table2array(data_true_POSE(ii,9:12))'; %orientation
        xtk(14:16,1) = table2array(data_true_POSE(ii,6:8))'; %position
        sigma.px = pixelsigma;
        
        % process left camera measurements
        [img1, map] = CameraObs(xtk, sigma, map, IMU2Cam);
        ImgPtsU(ImgInd,:) = img1.feats(1,:);
        ImgPtsV(ImgInd,:) = img1.feats(2,:);
        ImgPtsUm(ImgInd,:) = img1.feats_meas(1,:);
        ImgPtsVm(ImgInd,:) = img1.feats_meas(2,:);
        ImgId(ImgInd,:) = img1.feat_id;
        
        % process right camera measurements
        [img2, map]= CameraObstereo(xtk, sigma, map, IMU2Cam2, img1);
        Img2PtsU(ImgInd,:) = img2.feats(1,:);
        Img2PtsV(ImgInd,:) = img2.feats(2,:);
        Img2PtsUm(ImgInd,:) = img2.feats_meas(1,:);
        Img2PtsVm(ImgInd,:) = img2.feats_meas(2,:);
        Img2Id(ImgInd,:) = img2.feat_id;
        
        % timestamp
        ImgTS(ImgInd) = table2array(data_true_IMU(ii,3)) + 1e-9*table2array(data_true_IMU(ii,4)); % time in seconds
        
        %=================================================================
        % Our true pose of the IMU
        data.pose_true_gt(ImgInd,1:4) = table2array(data_true_POSE(ii,9:12)); %orientation
        data.pose_true_gt(ImgInd,5:7) = table2array(data_true_POSE(ii,6:8)); %position
        data.pose_true_gt(ImgInd,9) = ImgTS(ImgInd); % time in seconds
        % 4th element is always positive
        if data.pose_true_gt(ImgInd,4) < 0
            data.pose_true_gt(ImgInd,1:4) = -data.pose_true_gt(ImgInd,1:4);
        end
        %=================================================================
        
        % move forward in time
        ImgInd = ImgInd + 1;
        
        
    end

end

%% SHOW MAPPOINT WITH TRAJECTORY
%load data/map.mat;
if bool_show_map_waypoints
    figure('name', 'Point map and Waypoints');
    plot3(map.feats(1,:),map.feats(2,:),map.feats(3,:),'.'); hold on;
    % plot trajectory
    plot3(table2array(data_true_POSE(1:end,6)),table2array(data_true_POSE(1:end,7)),table2array(data_true_POSE(1:end,8)),'-'); hold on;
    % plot origin
    plot3(0,0,0,'*'); hold on;
    % plot the origin axis
    plot3([0 2], [0 0], [0, 0], 'r-'); hold on;
    plot3([0 0], [0 2], [0, 0], 'g-'); hold on;
    plot3([0 0], [0 0], [0, 2], 'b-'); hold on;
    legend('Map Points','Waypoints','Origin','x axis', 'y axis', 'z axis','Location','northwest');
    axis equal;
    xlabel('x(m)');ylabel('y(m)');zlabel('z(m)');
    title('Simulated Point Map and Waypoints');
    grid on;
end









%% generate monte carlo datasets

close all;
clear all;

%group of data
n_mc = 50;

% Base folder we should save too
basefolder = './GAZEBO_FREQ_100/';

for iter=0:n_mc-1
    
    % Clear all old variables
    clearvars -except n_mc iter basefolder    
    
    %run the script
    SCRIPT_gazebo_to_sim;

    %% Debug message
    fprintf('Writing data files for dataset %d...\n', iter);

    %make the dir for storing the data
    string_folder = [basefolder 'rawdata_' num2str(iter,'%02.f')];
    string_mkdir = ['mkdir ' string_folder ';'];
    eval(string_mkdir);

    % prepare to save the dat data file
    str_w = 'w+';
    str_cam_true = [string_folder '/camera_data_true.dat'];
    str_cam_meas = [string_folder '/camera_data_meas.dat'];
    str_cam_id = [string_folder '/camera_data_id.dat'];
    str_pose_true = [string_folder '/pose_true_gt.dat'];
    str_fid1 = ['fid1 = fopen( ''' str_cam_true ''',''' str_w ''' );'];
    str_fid2 = ['fid2 = fopen( ''' str_cam_meas ''',''' str_w ''' );'];
    str_fid3 = ['fid3 = fopen( ''' str_cam_id ''',''' str_w ''' );'];
    str_fid4 = ['fid4 = fopen( ''' str_pose_true ''',''' str_w ''' );'];
    eval(str_fid1);
    eval(str_fid2);
    eval(str_fid3);
    eval(str_fid4);
    for i=1:size(ImgPtsU,1)
        fprintf(fid1, '%s ', num2str(ImgPtsU(i,:), '%.14f '));
        fprintf(fid1, '%s ', num2str(ImgPtsV(i,:), '%.14f '));
        fprintf(fid1, '%s\n', num2str(ImgTS(i)*1000, '%.14f '));
        fprintf(fid2, '%s ', num2str(ImgPtsUm(i,:), '%.14f '));
        fprintf(fid2, '%s ', num2str(ImgPtsVm(i,:), '%.14f '));
        fprintf(fid2, '%s\n', num2str(ImgTS(i)*1000, '%.14f '));
        fprintf(fid3, '%s\n', num2str(ImgId(i,:), '%g '));
        fprintf(fid4, '%s\n', num2str([data.pose_true_gt(i,1:end-1) data.pose_true_gt(i,end)*1000], '%.14f '));
    end
    fclose(fid1);
    fclose(fid2);
    fclose(fid3);
    fclose(fid4);

    str_cam2_true = [string_folder '/camera2_data_true.dat'];
    str_cam2_meas = [string_folder '/camera2_data_meas.dat'];
    str_cam2_id = [string_folder '/camera2_data_id.dat'];
    str_fid11 = ['fid11 = fopen( ''' str_cam2_true ''',''' str_w ''' );'];
    str_fid21 = ['fid21 = fopen( ''' str_cam2_meas ''',''' str_w ''' );'];
    str_fid31 = ['fid31 = fopen( ''' str_cam2_id ''',''' str_w ''' );'];
    eval(str_fid11);
    eval(str_fid21);
    eval(str_fid31);
    for i=1:size(Img2PtsU,1)
        fprintf(fid11, '%s ', num2str(Img2PtsU(i,:), '%.14f '));
        fprintf(fid11, '%s ', num2str(Img2PtsV(i,:), '%.14f '));
        fprintf(fid11, '%s\n', num2str(ImgTS(i)*1000, '%.14f '));
        fprintf(fid21, '%s ', num2str(Img2PtsUm(i,:), '%.14f '));
        fprintf(fid21, '%s ', num2str(Img2PtsVm(i,:), '%.14f '));
        fprintf(fid21, '%s\n', num2str(ImgTS(i)*1000, '%.14f '));
        fprintf(fid31, '%s\n', num2str(Img2Id(i,:), '%g '));
    end
    fclose(fid11);
    fclose(fid21);
    fclose(fid31);

    % Write the imu data files
    str_imu_true = [string_folder '/imu_data_true.dat'];
    str_imu_meas = [string_folder '/imu_data_meas.dat'];
    str_pose_true = [string_folder '/pose_true.dat'];
    str_fid4 = ['fid4 = fopen( ''' str_imu_true ''',''' str_w ''' );'];
    str_fid5 = ['fid5 = fopen( ''' str_imu_meas ''',''' str_w ''' );'];
    str_fid6 = ['fid6 = fopen( ''' str_pose_true ''',''' str_w ''' );'];
    eval(str_fid4);
    eval(str_fid5);
    eval(str_fid6);
    for i=1:size(data.imu_true,1)
        fprintf(fid4, '%s\n', num2str([data.imu_true(i,1:end-1) data.imu_true(i,end)*1000], '%.14f '));
        fprintf(fid5, '%s\n', num2str([data.imu_meas(i,1:end-1) data.imu_meas(i,end)*1000], '%.14f '));
        fprintf(fid6, '%s\n', num2str([data.pose_true_imu(i,1:end-1) data.pose_true_imu(i,end)*1000], '%.14f '));
    end
    fclose(fid4);
    fclose(fid5);
    fclose(fid6);
    
    % extrinsic transforms
    str_IMU2Cam = [string_folder '/IMU2Cam.dat'];
    str_fid8 = ['fid8 = fopen( ''' str_IMU2Cam ''',''' str_w ''' );'];
    eval(str_fid8);
    fprintf(fid8, '%s ', num2str(IMU2Cam.q', '%.14f '));
    fprintf(fid8, '%s ', num2str(IMU2Cam.p', '%.14f '));
    fprintf(fid8, '%s ', num2str(IMU2Cam2.q', '%.14f '));
    fprintf(fid8, '%s ', num2str(IMU2Cam2.p', '%.14f '));
    fclose(fid8);
    
    % finally our sigmas
    str_Sigma = [string_folder '/Sigma.dat'];
    str_fid10 = ['fid10 = fopen( ''' str_Sigma ''',''' str_w ''' );'];
    eval(str_fid10);
    fprintf(fid10, '%s ', num2str(gyroscope_noise_density, '%.14f '));
    fprintf(fid10, '%s ', num2str(gyroscope_random_walk, '%.14f '));
    fprintf(fid10, '%s ', num2str(accelerometer_noise_density, '%.14f '));
    fprintf(fid10, '%s ', num2str(accelerometer_random_walk, '%.14f '));
    fprintf(fid10, '%s ', num2str(pixelsigma, '%.14f '));
    fclose(fid10);
    
    
end








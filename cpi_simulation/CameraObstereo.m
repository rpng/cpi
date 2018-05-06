function [img2, map]= CameraObstereo(xtk, sigma, map, IMU2Cam2, img1)
% % generate camera observations %%
%******this is used for generating the obeservation at each time steps 

img2.feats = zeros(2,map.nfeats_per_im);
img2.feats_meas = zeros(2,map.nfeats_per_im);
img2.feat_id = zeros(1,map.nfeats_per_im);

% global to imu
%**********Imu position in global frame
g_P_i = xtk(14:16,1);
%**********Imu rotation refer to global frame I2G_R
g_C_i = quat2rot(xtk(1:4,1))';

% i_P_g = - g_C_i' * g_P_i;

% imu to laser
%********cam center in the IMU frame (fixed)
i_P_c = IMU2Cam2.p;
%********cam rotation refer to the IMU frame (fixed)
i_C_c = quat2rot(IMU2Cam2.q);

g_P_f_v = map.feats(:,img1.feat_id);
g_P_i_v = repmat(g_P_i, 1, map.nfeats_per_im);
i_P_c_v = repmat(i_P_c,1, map.nfeats_per_im);

% fprintf('g_P_f_v: %d, %d\n', size(g_P_f_v,1), size(g_P_f_v,2));
% fprintf('g_P_i_v: %d, %d\n', size(g_P_i_v,1), size(g_P_i_v,2));
% fprintf('i_P_c_v: %d, %d\n', size(i_P_c_v,1), size(i_P_c_v,2));

baseline = i_P_c(1);

p_in_Cam = i_C_c' * g_C_i' * (g_P_f_v - g_P_i_v - g_C_i * i_P_c_v);

for i=1:map.nfeats_per_im
    %img2.feats(:,i) = [p_in_Cam(1,i)/p_in_Cam(3,i); p_in_Cam(2,i)/p_in_Cam(3,i)];
    img2.feats(:,i) = [img1.feats(1,i) - baseline/p_in_Cam(3,i);img1.feats(2,i)];
    %compare = [p_in_Cam(1,i)/p_in_Cam(3,i); p_in_Cam(2,i)/p_in_Cam(3,i)];
    img2.feats_meas(:,i) = img2.feats(:,i) + sigma.px * randn(2,1);
end

img2.feat_id = img1.feat_id;



% %*********p_in_cam is all the feats coordinates in 3D space, dim:3 X nfeats_valid
% p_in_Cam = i_C_c' * g_C_i' * ( map.feats(:,1:map.nfeats_valid) - repmat(g_P_i, 1, map.nfeats_valid) - g_C_i * repmat(i_P_c, 1, map.nfeats_valid));
% 
% %********what does jpt mean? 
% jpt = 1;
% for i = 1:map.nfeats_valid
%     %***********
%     theta = acos(p_in_Cam(3,i)/norm(p_in_Cam(:,i))) * 180/pi;
%     
%     %%%%%%%%%%%
%     if (abs(theta) < 22.5) && (p_in_Cam(3,i) > 0) %% 45 deg fov, and make sure that the feature points are in front of the camera%%        
%         img.feats(:,jpt) = [p_in_Cam(1,i)/p_in_Cam(3,i) ; p_in_Cam(2,i)/p_in_Cam(3,i)]; %******not related to intrinsic parameters and distortion
%         %*******why add noises in this way? 
%         img.feats_meas(:,jpt) = img.feats(:,jpt) + sigma.px * randn(2,1); %********add noises
%         img.feat_id(jpt) = i; %%%store the id for recognization 
%         jpt = jpt + 1;
%     end
%     
%     if jpt > map.nfeats_per_im
%         break;
%     end
% end

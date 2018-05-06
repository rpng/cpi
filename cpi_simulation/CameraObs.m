function [img, map] = CameraObs(xtk, sigma, map, IMU2Cam)
% % generate camera observations %%
%******this is used for generating the obeservation at each time steps 

img.feats = zeros(2,map.nfeats_per_im);
img.feats_meas = zeros(2,map.nfeats_per_im);
img.feat_id = zeros(1,map.nfeats_per_im);

% global to imu
%**********Imu position in global frame
g_P_i = xtk(14:16,1);
%**********Imu rotation refer to global frame
g_C_i = quat2rot(xtk(1:4,1))';

% i_P_g = - g_C_i' * g_P_i;

% imu to laser
%********cam center in the IMU frame (fixed)
i_P_c = IMU2Cam.p;
%********cam rotation refer to the IMU frame (fixed)
i_C_c = quat2rot(IMU2Cam.q);

%*********p_in_cam is all the feats coordinates in 3D space, dim:3 X nfeats_valid
p_in_Cam = i_C_c' * g_C_i' * ( map.feats(:,1:map.nfeats_valid) - repmat(g_P_i, 1, map.nfeats_valid) - g_C_i * repmat(i_P_c, 1, map.nfeats_valid));

%********what does jpt mean?
jpt = 1;
ids = 1:map.nfeats_valid;
for i = ids % sequential feature ordering
%for i = ids(randperm(length(ids))) % random shuffle features
    %***********
    theta = acos(p_in_Cam(3,i)/norm(p_in_Cam(:,i))) * 180/pi;
    
    %%%%%%%%%%%
    if (abs(theta) < 50) && (p_in_Cam(3,i) > 0.5) %&& (norm(p_in_Cam(3,i)) < 10) %% 45 deg fov, and make sure that the feature points are in front of the camera%%        
        img.feats(:,jpt) = [p_in_Cam(1,i)/p_in_Cam(3,i) ; p_in_Cam(2,i)/p_in_Cam(3,i)]; %******not related to intrinsic parameters and distortion
        %*******why add noises in this way? 
        img.feats_meas(:,jpt) = img.feats(:,jpt) + sigma.px * randn(2,1); %********add noises
        img.feat_id(jpt) = i; %%%store the id for recognization 
        jpt = jpt + 1;
    end
    
    if jpt > map.nfeats_per_im
        break;
    end
end

    


%********if the feature points are not enough for nfeats_per_im, we will
%********randomly generate new points and add to the feature list?  
if jpt <= map.nfeats_per_im
    
    % if not enough points
    fprintf('Not enough points, need %d more\n',map.nfeats_per_im - jpt);
    fprintf('Please increase points \n');

    
    for k = jpt:map.nfeats_per_im        
        % % randomly generate a new point
        
%         theta = (45 * rand - 22.5) * pi/180;
%         phi = (45 * rand - 22.5) * pi/180;        
        theta = (60 * rand - 30) * pi/180;
        phi = (60 * rand - 30) * pi/180;
        
        %*********this 3X1 vector
        punit = roty(theta) * rotx(phi) * [0;0;1];  %unit point
        g_punit = g_C_i * i_C_c * punit;  %transfer the unit coordinate to global coordinate
        
        
        %********cylinder tweak ?? what does that mean? 
        %%% CYLINDER TWEAK %%%q
        D = g_punit(1:2);
        O = g_P_i(1:2);
        
        a = D'*D;
        b = 2 * O' * D;
        c = O' * O - (6)^2;
        
        if a==0
            alpha = -c/b;
        else
            alpha(1) = (-b+sqrt(b^2-4*a*c))/(2*a);
            alpha(2) = (-b-sqrt(b^2-4*a*c))/(2*a);
        end
        alpha = max(alpha);
        
        c = O' * O - (4.5)^2;
        if a==0
            beta = -c/b;
        else
            beta(1) = (-b+sqrt(b^2-4*a*c))/(2*a);
            beta(2) = (-b-sqrt(b^2-4*a*c))/(2*a);
        end
        beta = max(beta);
        if 1 %~isreal(beta)
            beta = inf;
        end
        
        %************************don't understand what the cylinder treak
        %*******used for? ?????? 
        %******but the output for the cylinder treak is the magnitude for
        %*******unit vector
        my_dist = min([alpha beta]);
        
        %********this is also 3X1 vector 
        p_in_Cam = my_dist * punit; %(10 * rand + 1) * punit; % * rand + .1)
        
        map.nfeats_valid = map.nfeats_valid + 1;
        %********dynamically increase the dimension of the map.feats
        map.feats(:,map.nfeats_valid) = g_P_i + g_C_i * i_P_c + g_C_i * i_C_c * p_in_Cam;
        
        %********generate the needed feature point in the image plane
        img.feats(:,k) = [p_in_Cam(1)/p_in_Cam(3) ; p_in_Cam(2)/p_in_Cam(3)];
        %********add noise to teh feature points. 
        img.feats_meas(:,k) = img.feats(:,k) + sigma.px * randn(2,1);
        
        %********add recognization id
        img.feat_id(k) = map.nfeats_valid;
    end
    
end


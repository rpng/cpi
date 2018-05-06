function [xek_1k,Phi,Pkk] = IMU_prop4(xekk,Pkk,Imu,GYRO,ACCEL,params)

DT = params.DT;
% sxekk - STATE ESTIMATES (16 state estimates  propagated - 15 independent)
%
% $LastChangedDate: 2008-07-31 21:29:18 -0500 (Thu, 31 Jul 2008) $
% $Revision: 449 $
% Faraz Mirzaei 
% faraz@cs.umn.edu
%
% from GLOBAL to LOCAL - ORIENTATION
% 1 q1
% 2 q2
% 3 q3
% 4 q4
%
% SENSOR BIAS - GYRO
% 5 gyro_bias a
% 6 gyro_bias b
% 7 gyro_bias c
%
% wrt GLOBAL - VELOCITY
% 8 v1
% 9 v2
% 10 v3
%
% SENSOR BIAS - ACCEL
% 11 accel_bias a
% 12 accel_bias b
% 13 accel_bias c
%
% wrt GLOBAL - POSITION
% 14 r1
% 15 r2
% 16 r3
%
% LOCAL Camera_wrt_IMU Calibration - QUATERNION
% 17 q1_c
% 18 q2_c
% 19 q3_c
% 20 q4_c
%
% LOCAL Camera_wrt_IMU Calibration - TRANSLATION
% 21 px_c
% 22 py_c
% 23 pz_c
%
%-----------------------------------------------------------------------
%
% Pkk - COVARIANCE ESTIMATES 
%
% FROM GLOBAL TO LOCAL - ORIENTATION ERROR
% 1 dq1 *2 = d(theta1)
% 2 dq2 *2 = d(theta2)
% 3 dq3 *2 = d(theta3)
%
% SENSOR BIAS - GYRO ERROR
% 4 D(gyro_bias a)
% 5 D(gyro_bias b)
% 6 D(gyro_bias c)
%
% wrt GLOBAL VELOCITY ERROR
% 7 D(v1)
% 8 D(v2)
% 9 D(v3)
%
% SENSOR BIAS - ACCEL ERROR
% 10 D(accel_bias a)
% 11 D(accel_bias b)
% 12 D(accel_bias c)
%
% wrt GLOBAL - POSITION ERROR
% 13 D(r1)
% 14 D(r2)
% 15 D(r3)
%
% LOCAL Camera_wrt_IMU Calibration - QUATERNION
% 16 dq1_c *2 = d(theta_c1)
% 17 dq2_c *2 = d(theta_c2)
% 18 dq3_c *2 = d(theta_c3)
%
% LOCAL Camera_wrt_IMU Calibration - TRANSLATION
% 19 D(px_c)
% 20 D(py_c)
% 21 D(pz_c)
%

% sigma.r,sigma.w,sigma.g,sigma.v

% interpolator's IDs
Q_ID = 1;
OMEGA_ID = 2;
VDOT_ID = 3;
PHI11_ID = 4;
PHI12_ID = 5;

F33 = -2*skewsymm(params.earth.omega);
F35 = -skewsymm(params.earth.omega)^2;

% time steps 
N = size(GYRO,2)-1;
imu_time_nodes = 0:DT:DT*N;

%==========================================================================
% State Estimates Propagation
%==========================================================================

xek_1k = zeros(16,1);

%-------------------------------------
% GYRO BIAS PROPAGATION (PREDICTION)
%-------------------------------------
% Previous bias estimate - tk
gyro_bias_kk = xekk(5:7,1);

% Current bias estimate - tk+1
gyro_bias_k_1k = gyro_bias_kk;

% write to the state vector
xek_1k(5:7,1) = gyro_bias_k_1k;

% GYRO de-bias
wk = GYRO - gyro_bias_kk*ones(1,N+1);
% wk_1 = GYRO(:,2) - gyro_bias_kk;

%--------------------------------------
% ACCEL BIAS PROPAGATION (PREDICTION)
%--------------------------------------
% Previous bias estimate - tk
accel_bias_kk = xekk(11:13,1);

% Current bias estimate - tk+1
accel_bias_k_1k = accel_bias_kk;

% write to the state vector
xek_1k(11:13,1) = accel_bias_k_1k;

% de-bias ACCEL
dotVk = ACCEL - accel_bias_kk*ones(1,N+1);
% dotVk_1 = ACCEL(:,2) - accel_bias_kk;

%-------------------------------------------------
% ORIENTATION, VELOCITY and POSITION PROPAGATION 
%-------------------------------------------------

qvp_old = xekk([1:4 8:10 14:16],1);

% ODE solver
qkk = zeros(4,N+1);
qkk(:,1) = xekk(1:4,1);

for i = 1:N
    qvp_old = rk4(@statedot,[(i-1)*DT,i*DT],qvp_old);
    % normalize quaternion
    qvp_old(1:4) = qvp_old(1:4)/norm(qvp_old(1:4));
    qkk(:,i+1) = qvp_old(1:4);
end

% % normalize (just for numerical stability)
% qk_1k = state_k_1k(1:4);
% qk_1k = qk_1k/norm(qk_1k);

% write to the state vector
xek_1k(1:4,1) = qvp_old(1:4,end);
xek_1k(8:10,1) = qvp_old(5:7,end);
xek_1k(14:16,1) = qvp_old(8:10,end);


if xek_1k(4,1) < 0
    xek_1k(1:4,1) = -xek_1k(1:4,1);
end


%==========================================================================
% Jacobian computation
%==========================================================================

%---------
% Phi 11
%---------

Phi11 = zeros(9,N+1);
% initial condition
Phi11(1,1) = 1; Phi11(5,1) = 1; Phi11(9,1) = 1;

% ODE solver
for i = 1:N
    Phi11(:,i+1) = rk4(@Phi11dot,[(i-1)*DT,i*DT],Phi11(:,i));
end

Phi11_f = reshape(Phi11(:,end),3,3);

%-------------
% Phi 31 & 51
%-------------

% initial condition
Phi3151 = zeros(18,1);

% ODE solver
for i = 1:N
    Phi3151 = rk4(@Phi3151dot,[(i-1)*DT,i*DT],Phi3151);
end

% extract solution
Phi31_f = reshape(Phi3151(1:9),3,3);
Phi51_f = reshape(Phi3151(10:18),3,3);

%--------
% Phi 12
%--------

% initial condition
Phi12 = zeros(9,N+1);
% Phi12(:,i) = zeros(9,1);

% ODE solver
for i = 1:N
    Phi12(:,i+1) = rk4(@Phi12dot,[(i-1)*DT,i*DT],Phi12(:,i));
end

Phi12_f = reshape(Phi12(:,end),3,3);

%-------------
% Phi 32 & 52
%-------------

% initial condition
Phi3252 = zeros(18,1);

% ODE solver
for i = 1:N
    Phi3252 = rk4(@Phi3252dot,[(i-1)*DT,i*DT],Phi3252);
end

% extract solution
Phi32_f = reshape(Phi3252(1:9),3,3);
Phi52_f = reshape(Phi3252(10:18),3,3);

%-------------
% Phi 33 & 53
%-------------

% initial condition
Phi3353 = zeros(18,1);
Phi3353(1) = 1; Phi3353(5) = 1; Phi3353(9) = 1;

% ODE solver
for i = 1:N
    Phi3353 = rk4(@Phi3353dot,[(i-1)*DT,i*DT],Phi3353);
end

% extract solution
Phi33_f = reshape(Phi3353(1:9),3,3);
Phi53_f = reshape(Phi3353(10:18),3,3);

%-------------
% Phi 34 & 54
%-------------

% initial condition
Phi3454 = zeros(18,1);

% ODE solver
for i = 1:N
    Phi3454 = rk4(@Phi3454dot,[(i-1)*DT,i*DT],Phi3454);
end

% extract solution
Phi34_f = reshape(Phi3454(1:9,end),3,3);
Phi54_f = reshape(Phi3454(10:18,end),3,3);

%-------------
% Phi 35 & 55
%-------------

% initial condition
Phi3555 = zeros(18,1);
Phi3555(10) = 1; Phi3555(14) = 1; Phi3555(18) = 1;

% ODE solver
for i = 1:N
    Phi3555 = rk4(@Phi3555dot,[(i-1)*DT,i*DT],Phi3555);
end

% extract solution
Phi35_f = reshape(Phi3555(1:9),3,3);
Phi55_f = reshape(Phi3555(10:18),3,3);



%

Phi = eye(15,15);

Phi(1:3,1:3) = Phi11_f;
Phi(7:9,1:3) = Phi31_f;
Phi(13:15,1:3) = Phi51_f;
Phi(1:3,4:6) = Phi12_f;
Phi(7:9,4:6) = Phi32_f;
Phi(13:15,4:6) = Phi52_f;
Phi(7:9,7:9) = Phi33_f;
Phi(13:15,7:9) = Phi53_f;
Phi(7:9,10:12) = Phi34_f;
Phi(13:15,10:12) = Phi54_f;
Phi(7:9,13:15) = Phi35_f;
Phi(13:15,13:15) = Phi55_f;

%==========================================================================
% Covariance Matrix
%==========================================================================

Q = diag([Imu.sr^2*[1 1 1] ...
    Imu.sw^2*[1 1 1] ...
    Imu.sg^2*[1 1 1] ...
    Imu.sv^2*[1 1 1] ...
    [0 0 0]]);

for i = 1:N
    Pkk = rk4(@Pdot,[(i-1)*DT,i*DT],Pkk);
end
%Pkk = (Pkk+Pkk')/2;
%==========================================================================
% Internal functions
%==========================================================================

%---------
% States
%---------

    function state_out = statedot(t,state)
        state_out = zeros(10,1);
        quat = state(1:4);%/norm(state(1:4));
        state_out(1:4) = 0.5*Omega_calculation(lin_interp(t,OMEGA_ID)-...
            quat2rot(quat)*params.earth.omega)*quat;

        quat = quat/norm(quat);
        if quat(4,1) < 0
            quat = -quat;
        end

        state_out(5:7) = quat2rot(quat)'*(lin_interp(t,VDOT_ID)) ...
            -skewsymm(params.earth.omega)*(2*state(5:7)+skewsymm(params.earth.omega)*params.earth.p) ...
            + params.earth.g;

        state_out(8:10) = state(5:7);
    end

%---------------------------
% Jacobians: Phi 11, 31, 51
%---------------------------

    function Phi_out = Phi11dot(t,Phi_in)
        Phi_out = zeros(9,1);
        Phi_out(1:3) = -skewsymm(lin_interp(t,OMEGA_ID))*Phi_in(1:3);
        Phi_out(4:6) = -skewsymm(lin_interp(t,OMEGA_ID))*Phi_in(4:6);
        Phi_out(7:9) = -skewsymm(lin_interp(t,OMEGA_ID))*Phi_in(7:9);
    end

    function Phi_out = Phi3151dot(t,Phi_in)
        Phi_out = zeros(18,1);
        quat_tmp = lin_interp(t,Q_ID);
        quat_tmp = quat_tmp/norm(quat_tmp);
        F31 = -quat2rot(quat_tmp)'*...
            skewsymm(lin_interp(t,VDOT_ID));
        Phi11_tmp = lin_interp(t,PHI11_ID);
        Phi_out(1:3) = F31*Phi11_tmp(1:3)+F33*Phi_in(1:3)+F35*Phi_in(10:12);
        Phi_out(4:6) = F31*Phi11_tmp(4:6)+F33*Phi_in(4:6)+F35*Phi_in(13:15);
        Phi_out(7:9) = F31*Phi11_tmp(7:9)+F33*Phi_in(7:9)+F35*Phi_in(16:18);
        Phi_out(10:12) = Phi_in(1:3);
        Phi_out(13:15) = Phi_in(4:6);
        Phi_out(16:18) = Phi_in(7:9);
    end

%---------------------------
% Jacobians: Phi 12, 32, 52
%---------------------------

    function Phi_out = Phi12dot(t,Phi_in)
        Phi_out = zeros(9,1);
        Phi_out(1:3) = -skewsymm(lin_interp(t,OMEGA_ID))*Phi_in(1:3) - [1;0;0];
        Phi_out(4:6) = -skewsymm(lin_interp(t,OMEGA_ID))*Phi_in(4:6) - [0;1;0];
        Phi_out(7:9) = -skewsymm(lin_interp(t,OMEGA_ID))*Phi_in(7:9) - [0;0;1];
    end


    function Phi_out = Phi3252dot(t,Phi_in)
        Phi_out = zeros(18,1);
        quat_tmp = lin_interp(t,Q_ID);
        quat_tmp = quat_tmp/norm(quat_tmp);
        F31 = -quat2rot(quat_tmp)'*...
            skewsymm(lin_interp(t,VDOT_ID));
        Phi12_tmp = lin_interp(t,PHI12_ID);
        Phi_out(1:3) = F31*Phi12_tmp(1:3)+F33*Phi_in(1:3)+F35*Phi_in(10:12);
        Phi_out(4:6) = F31*Phi12_tmp(4:6)+F33*Phi_in(4:6)+F35*Phi_in(13:15);
        Phi_out(7:9) = F31*Phi12_tmp(7:9)+F33*Phi_in(7:9)+F35*Phi_in(16:18);
        Phi_out(10:12) = Phi_in(1:3);
        Phi_out(13:15) = Phi_in(4:6);
        Phi_out(16:18) = Phi_in(7:9);
    end

%-----------------------
% Jacobians: Phi 33, 53
%-----------------------

    function Phi_out = Phi3353dot(t,Phi_in)
        Phi_out = zeros(18,1);
        Phi_out(1:3) = F33*Phi_in(1:3)+F35*Phi_in(10:12);
        Phi_out(4:6) = F33*Phi_in(4:6)+F35*Phi_in(13:15);
        Phi_out(7:9) = F33*Phi_in(7:9)+F35*Phi_in(16:18);
        Phi_out(10:12) = Phi_in(1:3);
        Phi_out(13:15) = Phi_in(4:6);
        Phi_out(16:18) = Phi_in(7:9);        
    end

%-----------------------
% Jacobians: Phi 34, 54
%-----------------------

    function Phi_out = Phi3454dot(t,Phi_in)
        Phi_out = zeros(18,1);
        quat_tmp = lin_interp(t,Q_ID);
        quat_tmp = quat_tmp/norm(quat_tmp);
        F34 = -quat2rot(quat_tmp)';
        Phi_out(1:3) = F34(:,1);
        Phi_out(4:6) = F34(:,2);
        Phi_out(7:9) = F34(:,3);
        Phi_out(10:12) = Phi_in(1:3);
        Phi_out(13:15) = Phi_in(4:6);
        Phi_out(16:18) = Phi_in(7:9);
    end


%-----------------------
% Jacobians: Phi 35, 55
%-----------------------

    function Phi_out = Phi3555dot(t,Phi_in)
        Phi_out = zeros(18,1);
        Phi_out(1:3) = F33*Phi_in(1:3)+F35*Phi_in(10:12);
        Phi_out(4:6) = F33*Phi_in(4:6)+F35*Phi_in(13:15);
        Phi_out(7:9) = F33*Phi_in(7:9)+F35*Phi_in(16:18);
        Phi_out(10:12) = Phi_in(1:3);
        Phi_out(13:15) = Phi_in(4:6);
        Phi_out(16:18) = Phi_in(7:9);        
    end

%-----------------------

    function P_out = Pdot(t,P_in)
        quat_tmp = lin_interp(t,Q_ID);
        quat_tmp = quat_tmp/norm(quat_tmp);
        
        F11 = -skewsymm(lin_interp(t,OMEGA_ID));
        F31 = -quat2rot(quat_tmp)'*...
            skewsymm(lin_interp(t,VDOT_ID));
        F34 = -quat2rot(quat_tmp)';
        
        F = zeros(15,15);
        F(1:3,1:3) = F11;
        F(1:3,4:6) = -eye(3);
        F(7:9,1:3) = F31;
        F(7:9,7:9) = F33;
        F(7:9,10:12) = F34;
        F(7:9,13:15) = F35;
        F(13:15,7:9) = eye(3);
        
        P_out = F*P_in + P_in*F' + Q;
    end

%-------------
% Integrators
%-------------

%     function y = rk2(rgfun,tspan,y0)
%         % second order Runge-Kutta
%         h = tspan(2)-tspan(1);
%         k1 = h*rgfun(tspan(1),y0);
%         y = y0 + h*rgfun(tspan(1)+0.5*h, y0+0.5*k1);
%     end

    function y = rk4(rgfun,tspan,y0)
        % Fourth order Runge-Kutta
        h = tspan(2)-tspan(1);
        k1 = h*rgfun(tspan(1),y0);
        k2 = h*rgfun(tspan(1)+h/2,y0+k1/2);
        k3 = h*rgfun(tspan(1)+h/2,y0+k2/2);
        k4 = h*rgfun(tspan(1)+h,y0+k3);
        y = y0 + (k1+2*k2+2*k3+k4)/6;
    end

%---------------
% Interpolators
%---------------

    function Yt = lin_interp(t,id)
        %ind1 = floor(t/DT)+1; doesn't work (epsilon problem)
        %ind2 = ceil(t/DT)+1;
        %offset = mod(t,DT);
                
        ind1 = find(imu_time_nodes <= t, 1, 'last' );
        ind2 = find(imu_time_nodes >= t, 1 );
        offset = t - imu_time_nodes(ind1);
        
        switch id
            case Q_ID,
                INTP_quat_diff = quat_mul(qkk(:,ind2),quat_inv(qkk(:,ind1)));
                INTP_quat_inc = [INTP_quat_diff(1:3)*(offset/DT) ; 1];
                INTP_quat_inc = INTP_quat_inc/norm(INTP_quat_inc);
                Yt = quat_mul(qkk(:,ind1),INTP_quat_inc);                
                %Yt = qkk(:,ind1) + (qkk(:,ind2)-qkk(:,ind1))*(offset/DT);
            case OMEGA_ID,
                Yt = wk(:,ind1) + (wk(:,ind2)-wk(:,ind1))*(offset/DT);
            case VDOT_ID,
                Yt = dotVk(:,ind1) + (dotVk(:,ind2)-dotVk(:,ind1))*(offset/DT);
            case PHI11_ID,
                Yt = Phi11(:,ind1) + (Phi11(:,ind2)-Phi11(:,ind1))*(offset/DT);
            case PHI12_ID,
                Yt = Phi12(:,ind1) + (Phi12(:,ind2)-Phi12(:,ind1))*(offset/DT);
            otherwise
                error('unknown ID in interpolator!')
        end
    end

end

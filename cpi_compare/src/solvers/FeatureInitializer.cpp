/**
 * MIT License
 * Copyright (c) 2018 Kevin Eckenhoff
 * Copyright (c) 2018 Patrick Geneva
 * Copyright (c) 2018 Guoquan Huang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#include "FeatureInitializer.h"



/**
 * Will create the vector of camera poses
 */
void FeatureInitializer::create_camera_poses() {

    // Resize our vector (4=quat, 3=pos)
    // Note since we start at nodect=0 we need +1 for size
    cam0poses.resize(7*(nodect+1));
    cam0poses.setZero();

    // Loop through each node in the state (our states go from 1 to nodect)
    // This means the first 7 of this vector should NOT be used!!! (they will be all zero)
    for (size_t i=0; i <= nodect; i++){
        // Only get it if we have that key
        if(!values.exists(X(i)))
            continue;
        // Get the current state
        Vector4d q_GtoI = values.at<gtsam::JPLNavState>(X(i)).q();
        Vector3d p_IinG = values.at<gtsam::JPLNavState>(X(i)).p();
        // Transform into camera 0 frame of reference
        cam0poses.block(7*i,0,4,1) = rot_2_quat(config->R_C0toI.transpose()*quat_2_Rot(q_GtoI));
        cam0poses.block(7*i+4,0,3,1) = p_IinG - quat_2_Rot(q_GtoI).transpose()*config->R_C0toI*config->p_IinC0;
    }

}




/**
 * Function that takes in a feature and poses to initialize its 3D position
 */
bool FeatureInitializer::initialize_feature(feature& feat) {

    // Get the feature ID
    //uint featid = (!feat.leftids.empty())? feat.leftids.at(0) : feat.rightids.at(0);

    // Triangulate the feature
    bool success = triangulate_feature(feat);
    //ROS_INFO("[FEAT]: feature #%d = %.4f, %.4f, %.4f (triangulated)",(int)featid,feat.pos_FinA_trianglated(0),feat.pos_FinA_trianglated(1),feat.pos_FinA_trianglated(2));
    //ROS_INFO("[FEAT]: feature #%d = %.4f, %.4f, %.4f (triangulated)",(int)featid,feat.pos_FinG(0),feat.pos_FinG(1),feat.pos_FinG(2));

    // Return if failure on optimization
    if(!success) {
        //ROS_WARN("[FEAT]: feature #%d triangulation failed",(int)featid);
        return false;
    }

    // Optimize the position using Gauss newton
    success = optimize_feature(feat);
    //ROS_INFO("[FEAT]: feature #%d = %.4f, %.4f, %.4f (optimized)",(int)featid,feat.pos_FinA_optimized(0),feat.pos_FinA_optimized(1),feat.pos_FinA_optimized(2));
    //ROS_INFO("[FEAT]: feature #%d = %.4f, %.4f, %.4f (optimized)",(int)featid,feat.pos_FinG(0),feat.pos_FinG(1),feat.pos_FinG(2));

    // Return if failure on optimization
    if(!success) {
        //ROS_WARN("[FEAT]: feature #%d optimization failed",(int)featid);
        return false;
    }


    // Calculate inverse depth for this feature
    Vector4d q_GtoA = values.at<gtsam::JPLNavState>(X(feat.leftstateids.at(0))).q();
    Vector3d p_AinG = values.at<gtsam::JPLNavState>(X(feat.leftstateids.at(0))).p();
    Vector3d feat_FinA = config->R_C0toI.transpose()*quat_2_Rot(q_GtoA)*(feat.pos_FinG-p_AinG) - config->R_C0toI*config->p_IinC0;
    feat.pos_FinA_inv << feat_FinA(0)/feat_FinA(2),feat_FinA(1)/feat_FinA(2),1/feat_FinA(2);


    // Return success!
    return true;
}





/**
 * Will triangulate the feature using least squares optimization
 */
bool FeatureInitializer::triangulate_feature(feature& feat) {

    // Calculate the anchor pose for this feature (favor the left side)
    size_t anchorid = (!feat.leftstateids.empty())? feat.leftstateids.at(0) : feat.rightstateids.at(0);

    // Our linear system matrices
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2*feat.leftstateids.size()+2*feat.rightstateids.size(), 3);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(2*feat.leftstateids.size()+2*feat.rightstateids.size(), 1);

    // Get the position of the anchor pose
    Eigen::Matrix<double,3,3> R_G_2_anc = quat_2_Rot(cam0poses.block(7*anchorid,0,4,1));
    Eigen::Matrix<double,3,1> p_anc_in_G = cam0poses.block(7*anchorid+4,0,3,1);

    // Location in the linear system matrices
    size_t c = 0;

    // Add LEFT features
    for (size_t m = 0; m < feat.leftstateids.size(); m++){
        // Get the state ID
        size_t i = feat.leftstateids.at(m);
        // DEBUG: Check that we are only using camera poses we have estimates for
        if(!values.exists(X(i))) {
            ROS_ERROR("TRYING TO TRIANGULATE FEATURE WITH MARGINALIZED POSE....");
        }
        // Get the position of this clone in the global
        Eigen::Matrix<double, 3, 1> p_i_in_G = cam0poses.block(7 * i + 4, 0, 3, 1);
        Eigen::Matrix<double, 3, 3> R_G_2_i = quat_2_Rot(cam0poses.block(7 * i, 0, 4, 1));
        // Convert current position relative to anchor
        Eigen::Matrix<double,3,1> p_i_in_anc= R_G_2_anc*(p_i_in_G-p_anc_in_G);
        Eigen::Matrix<double,3,3> R_anc_to_i= R_G_2_i*R_G_2_anc.transpose();
        // Get the UV coordinate normal
        Eigen::Matrix<double,3,1> b_i;
        b_i << feat.leftuv.at(m)(0), feat.leftuv.at(m)(1), 1;
        b_i= R_anc_to_i.transpose()*b_i;
        b_i= b_i/b_i.norm();
        Eigen::Matrix<double,2,3> Bperp = Eigen::MatrixXd::Zero(2,3);
        Bperp << -b_i(2,0), 0 , b_i(0,0), 0 , b_i(2,0), -b_i(1,0);
        // Append to our linear system
        A.block(2*c,0,2,3) = Bperp;
        b.block(2*c,0,2,1) = Bperp*p_i_in_anc;
        c++;
    }

    // Calculate the transform from the LEFT to RIGHT cameras
    Eigen::Matrix<double,3,3> R_LtoR = config->R_C1toI.transpose()*config->R_C0toI;
    Eigen::Matrix<double,3,1> p_RinL = config->p_IinC0 - R_LtoR.transpose()*config->p_IinC1;

    // Add RIGHT features
    for (size_t m = 0; m < feat.rightstateids.size(); m++){
        // Get the state ID
        size_t i = feat.rightstateids.at(m);
        // DEBUG: Check that we are only using camera poses we have estimates for
        if(!values.exists(X(i))) {
            ROS_ERROR("TRYING TO TRIANGULATE FEATURE WITH MARGINALIZED POSE....");
        }
        // Get the position of this clone in the global
        Eigen::Matrix<double, 3, 1> p_i_in_G = cam0poses.block(7 * i + 4, 0, 3, 1);
        Eigen::Matrix<double, 3, 3> R_G_2_i = quat_2_Rot(cam0poses.block(7 * i, 0, 4, 1));
        // Convert current position relative to anchor
        Eigen::Matrix<double,3,1> p_i_in_anc= R_G_2_anc*(p_i_in_G+R_G_2_i.transpose()*p_RinL-p_anc_in_G);
        Eigen::Matrix<double,3,3> R_anc_to_i= R_LtoR*R_G_2_i*R_G_2_anc.transpose();
        // Get the UV coordinate normal
        Eigen::Matrix<double,3,1> b_i;
        b_i << feat.rightuv.at(m)(0), feat.rightuv.at(m)(1), 1;
        b_i= R_anc_to_i.transpose()*b_i;
        b_i= b_i/b_i.norm();
        Eigen::Matrix<double,2,3> Bperp = Eigen::MatrixXd::Zero(2,3);
        Bperp << -b_i(2,0), 0 , b_i(0,0), 0 , b_i(2,0), -b_i(1,0);
        // Append to our linear system
        A.block(2*c,0,2,3) = Bperp;
        b.block(2*c,0,2,1) = Bperp*p_i_in_anc;
        c++;
    }

    // Solve the linear system
    Eigen::MatrixXd p_f = (A).colPivHouseholderQr().solve(b);

    //Check A and p_f
    Eigen::JacobiSVD<Eigen::MatrixXd> svd((A.block(0, 0, A.rows(), 3)), Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd singularValues;
    singularValues.resize(svd.singularValues().rows(), 1);
    singularValues = svd.singularValues();
    double condA = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);

    // Save the triangulated point
    feat.pos_FinA_trianglated = p_f;

    // Calculate transform into global frame of reference
    feat.pos_FinG = R_G_2_anc.transpose()*feat.pos_FinA_trianglated + p_anc_in_G;

    // If we have a bad condition number, or it is too close
    // Then set the flag for bad (i.e. set z-axis to nan)
    if (std::abs(condA) > 1000 || p_f(2,0) < 0.1 || p_f(2,0) > 40 ){
        return false;
    }
    // Else success!
    return true;
}



/**
 * Computes error for a single step of the gauss newton
 */
double FeatureInitializer::compute_stereo_error(std::vector<Eigen::Matrix<double,2,1>  > uv_vec,std::vector<int> cam_id, std::vector<int> cam_index, Eigen::MatrixXd rel_pose_vec, double alpha, double beta, double rho){
    // Calculate the transform from the LEFT to RIGHT cameras
    Eigen::Matrix<double,3,3> R_LtoR = config->R_C1toI.transpose()*config->R_C0toI;
    Eigen::Matrix<double,3,1> p_RinL = config->p_IinC0 - R_LtoR.transpose()*config->p_IinC1;
    // Total error
    double err = 0;
    //Computes the error of GN
    for (size_t m=0; m < uv_vec.size(); m++) {
        // Get if we should be right of left camera
        Eigen::Matrix<double,3,3> R_cl_2_c= cam_index[m]==0? Eigen::Matrix<double,3,3>::Identity() : R_LtoR;
        Eigen::Matrix<double,3,1> p_c_in_cl= cam_index[m]==0? Eigen::Matrix<double,3,1>::Zero() : p_RinL;
        //Compensate for cam to cam transform when using stereo
        Eigen::Matrix<double, 3, 3> R_n_to_i = quat_2_Rot(rel_pose_vec.block(7 * m, 0, 4, 1));
        Eigen::Matrix<double, 3, 1> p_n_in_i = -R_n_to_i*rel_pose_vec.block(7 * m + 4, 0, 3, 1);
        p_n_in_i= R_cl_2_c*(p_n_in_i-p_c_in_cl );
        R_n_to_i= R_cl_2_c*R_n_to_i;
        double hi1 = R_n_to_i(0, 0) * alpha + R_n_to_i(0, 1) * beta + R_n_to_i(0, 2) + rho * p_n_in_i(0, 0);
        double hi2 = R_n_to_i(1, 0) * alpha + R_n_to_i(1, 1) * beta + R_n_to_i(1, 2) + rho * p_n_in_i(1, 0);
        double hi3 = R_n_to_i(2, 0) * alpha + R_n_to_i(2, 1) * beta + R_n_to_i(2, 2) + rho * p_n_in_i(2, 0);
        Eigen::Matrix<double, 2, 1> z;
        z << hi1 / hi3, hi2 / hi3;
        Eigen::Matrix<double,2,1> res= uv_vec[m]- z;
        err+=pow(res.norm(),2);
    }
    return err;
}




/**
 * Will triangulate the feature using least squares optimization
 */
bool FeatureInitializer::optimize_feature(feature& feat) {


    // Convert into Kevin's data format
    std::vector<Eigen::Matrix<double,2,1>> uv_vec;
    std::vector<int> cam_id;
    std::vector<int> cam_index;

    for(size_t i=0;i<feat.leftstateids.size();i++) {
        cam_id.push_back((int)feat.leftstateids.at(i));
        uv_vec.push_back(feat.leftuv.at(i));
        cam_index.push_back(0);
    }
    for(size_t i=0;i<feat.rightstateids.size();i++) {
        cam_id.push_back((int)feat.rightstateids.at(i));
        uv_vec.push_back(feat.rightuv.at(i));
        cam_index.push_back(1);
    }

    // Calculate the transform from the LEFT to RIGHT cameras
    Eigen::Matrix<double,3,3> R_LtoR = config->R_C1toI.transpose()*config->R_C0toI;
    Eigen::Matrix<double,3,1> p_RinL = config->p_IinC0 - R_LtoR.transpose()*config->p_IinC1;

    //Get into inverse depth
    double rho = 1/feat.pos_FinA_trianglated(2);
    double alpha = feat.pos_FinA_trianglated(0)/feat.pos_FinA_trianglated(2);
    double beta = feat.pos_FinA_trianglated(1)/feat.pos_FinA_trianglated(2);

    // Optimization parameters
    double max_lam = 1e8;
    double lam = 1e-3;
    double eps = 10000;
    int runs = 0;
    int max_runs = 20;

    // Variables used in the optimization
    bool recompute = true;
    Eigen::Matrix<double,3,3> Hess = Eigen::MatrixXd::Zero(3,3);
    Eigen::Matrix<double,3,1> grad = Eigen::MatrixXd::Zero(3,1);

    //Computes relative pose vec in first poses frame
    Eigen::MatrixXd rel_pose_vec(7*uv_vec.size(),1);
    for (size_t m=0; m < uv_vec.size(); m++) {
        rel_pose_vec.block(7*m,0,4,1) = quat_multiply(cam0poses.block(7*cam_id[m],0,4,1), Inv(cam0poses.block(7*cam_id[0],0,4,1)));
        rel_pose_vec.block(7*m+4,0,3,1) = quat_2_Rot(cam0poses.block(7*cam_id[0],0,4,1))*(cam0poses.block(7*cam_id[m]+4,0,3,1)-cam0poses.block(7*cam_id[0]+4,0,3,1));
    }

    // Cost at the last iteration
    double cost_old = compute_stereo_error(uv_vec,cam_id, cam_index, rel_pose_vec,alpha,beta,rho);

    // Loop till we have either
    // 1. Reached our max iteration count
    // 2. System is unstable
    // 3. System has converged
    while (runs < max_runs && lam < max_lam && eps > 1e-6) {

        //Triggers a recomputation of jacobians/information/gradients
        if (recompute) {

            Hess.setZero();
            grad.setZero();

            double err = 0;

            for (size_t m = 0; m < uv_vec.size(); m++) {
                Eigen::Matrix<double,3,3> R_cl_2_c= cam_index[m]== 0? Eigen::Matrix<double,3,3>::Identity() : R_LtoR;
                Eigen::Matrix<double,3,1> p_c_in_cl= cam_index[m]== 0? Eigen::Matrix<double,3,1>::Zero() : p_RinL;

                //Compensate for cam to cam transform when using stereo
                Eigen::Matrix<double, 3, 3> R_n_to_i = quat_2_Rot(rel_pose_vec.block(7 * m, 0, 4, 1));
                Eigen::Matrix<double, 3, 1> p_n_in_i = -R_n_to_i*rel_pose_vec.block(7 * m + 4, 0, 3, 1);

                p_n_in_i= R_cl_2_c*(p_n_in_i-p_c_in_cl );
                R_n_to_i= R_cl_2_c*R_n_to_i;


                double hi1 = R_n_to_i(0, 0) * alpha + R_n_to_i(0, 1) * beta + R_n_to_i(0, 2) + rho * p_n_in_i(0, 0);
                double hi2 = R_n_to_i(1, 0) * alpha + R_n_to_i(1, 1) * beta + R_n_to_i(1, 2) + rho * p_n_in_i(1, 0);
                double hi3 = R_n_to_i(2, 0) * alpha + R_n_to_i(2, 1) * beta + R_n_to_i(2, 2) + rho * p_n_in_i(2, 0);

                Eigen::Matrix<double, 2, 1> z;
                z << hi1 / hi3, hi2 / hi3;

                double d_z1_d_alpha = (R_n_to_i(0, 0) * hi3 - hi1 * R_n_to_i(2, 0)) / (pow(hi3, 2));
                double d_z1_d_beta = (R_n_to_i(0, 1) * hi3 - hi1 * R_n_to_i(2, 1)) / (pow(hi3, 2));
                double d_z1_d_rho = (p_n_in_i(0, 0) * hi3 - hi1 * p_n_in_i(2, 0)) / (pow(hi3, 2));

                double d_z2_d_alpha = (R_n_to_i(1, 0) * hi3 - hi2 * R_n_to_i(2, 0)) / (pow(hi3, 2));
                double d_z2_d_beta = (R_n_to_i(1, 1) * hi3 - hi2 * R_n_to_i(2, 1)) / (pow(hi3, 2));
                double d_z2_d_rho = (p_n_in_i(1, 0) * hi3 - hi2 * p_n_in_i(2, 0)) / (pow(hi3, 2));

                Eigen::Matrix<double, 2, 3> H;
                H << d_z1_d_alpha, d_z1_d_beta, d_z1_d_rho,
                        d_z2_d_alpha, d_z2_d_beta, d_z2_d_rho;

                //Compute Jacobians
                Eigen::Matrix<double, 2, 1> res;
                res = uv_vec[m] - z;

                err += pow(res.norm(), 2);
                grad += H.transpose() * res;
                Hess += H.transpose() * H;
            }

        }

        //Solve Levenberg iteration

        Eigen::MatrixXd Hess_l= Hess;

        for (size_t r=0; r < (size_t)Hess.rows(); r++){
            Hess_l(r,r)*= (1.0+lam);
        }

        Eigen::Matrix<double,3,1> dx = Hess_l.colPivHouseholderQr().solve(grad);
        //Eigen::Matrix<double,3,1> dx = (Hess+lam*Eigen::MatrixXd::Identity(Hess.rows(), Hess.rows())).colPivHouseholderQr().solve(grad);

        //Check if error has gone down
        double cost = compute_stereo_error(uv_vec,cam_id, cam_index, rel_pose_vec,alpha+dx(0,0),beta+dx(1,0),rho+dx(2,0));

        //Check if converged
        if (cost <= cost_old && (cost_old-cost)/cost_old < 1e-8 ) {
            alpha += dx(0, 0);
            beta += dx(1, 0);
            rho += dx(2, 0);
            eps = 0;
            break;
        }

        //If cost is lowered, accept step
        if (cost <= cost_old) {
            recompute = true;
            cost_old = cost;
            alpha += dx(0, 0);
            beta += dx(1, 0);
            rho += dx(2, 0);
            runs++;
            lam= lam/10;
            eps = dx.norm();
        }
            //Else inflate lambda (try to make more stable)
        else {
            //cout << "lam- " << lam << endl;
            recompute = false;
            lam=lam*10;
            continue;
        }
    }

    //cout << "alpha- " << alpha << endl;
    //cout << "beta- " << beta << endl;
    //cout << "rho- " << rho << endl;

    //Revert to standard
    feat.pos_FinA_optimized(0) = alpha/rho;
    feat.pos_FinA_optimized(1) = beta/rho;
    feat.pos_FinA_optimized(2) = 1/rho;

    //Get tangent plane to x_hat
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(feat.pos_FinA_optimized);
    Eigen::MatrixXd Q = qr.householderQ();

    // Max baseline we have between poses
    double base_line_max = 0.0;

    // Check maximum baseline
    // Loop through the other clones to see what the max baseline is
    for (size_t k = 1; k < uv_vec.size(); k++) {
        Eigen::Matrix<double, 3, 1> pi;
        pi = rel_pose_vec.block(7*k+4,0,3,1);
        // Dot product camera pose and nullspace
        double base_line = ((Q.block(0,1,3,2)).transpose() * pi).norm();
        if (base_line > base_line_max)
            base_line_max = base_line;
    }


    // See if feature can be accepted
    // 1. Bad if the feature is to close, to faraway, stable?
    // 2. Or if we have reached max iterations
    // 3. Or if we unstable
    // 4. If the pixel is not in the center of the image (note the uv_vec is normalized)
    /*bool bad = (x_hat(2,0) < .2 || x_hat(2,0) > 10 || x_hat.norm() > 10 || (x_hat.norm() / base_line_max) > 100)
               && !force_update
               || runs == max_runs
               || lam > max_lam
               || (std::abs(config->cam0_K(0,0)*uv_vec[0](0,0)) < 50 && std::abs(config->cam0_K(1,1)*uv_vec[0](1,0)) < 50);*/

    bool bad;
    //bad = (x_hat(2,0) <.2 || x_hat(2, 0)> 10 || x_hat.norm()> 10 || (x_hat.norm()/ base_line_max) > 100) && !force_update || runs == max_runs
    //      || lam > max_lam || (std::abs(config->cam0_K(0,0)*uv_vec[0](0,0)) < 50 && std::abs(config->cam0_K(1,1)*uv_vec[0](1,0)) < 50);
    //bad= (x_hat(2,0) < .5 || (x_hat.norm()/ base_line_max) > 50);
    bad = feat.pos_FinA_optimized(2) < 0.1 || std::isnan(feat.pos_FinA_optimized.norm());

    // Move it back to global
    feat.pos_FinG = quat_2_Rot(cam0poses.block(7*cam_id[0],0,4,1)).transpose()*feat.pos_FinA_optimized+cam0poses.block(7*cam_id[0]+4,0,3,1);

    // Debug info
    //cout << "optimization runs = " << runs << " | cost = " << cost_old << " | lamda = " << lam << " | isnan = " << std::isnan(feat.pos_FinA_optimized.norm()) << endl;

    // Return if it is good (i.e. not bad)
    return !bad;
}


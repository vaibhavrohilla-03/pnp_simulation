#pragma once
#include "eigen3/Eigen/Eigen"

namespace Utils {

    Eigen::Matrix4d create_HomogenousMatrix(Eigen::Vector3d& pos, Eigen::Matrix3d& rotation_matrix) {
        
        Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
        result.block<3,3>(0,0) = rotation_matrix;
        result.block<3,1>(0,3) = pos;

        return result;

    }

    Eigen::Matrix<double, 6, 1> get_twist_error(double errors[2], Eigen::Matrix4d& matrix) {

        Eigen::Matrix<double, 6, 1> V_b;
        Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d p = matrix.block<3, 1>(0, 3);

        Eigen::AngleAxisd aa(R);

        double theta = aa.angle();
        Eigen::Vector3d w = aa.axis();

        Eigen::Vector3d w_theta = theta * w;
        V_b.head<3>() = w_theta;
        
        Eigen::Vector3d twist;
        if(theta < 1e-6) {
            Eigen::Vector3d w_cross = w_theta.cross(p);
            twist = p - 0.5 * w_cross;
        }

        else {
            Eigen::Vector3d w_cross = w_theta.cross(p);
            Eigen::Vector3d w_cross_w_p = w.cross(w_cross);  
            
            twist = p - 0.5 * theta * w_cross_w_p + 
                (1.0 - (theta * std::sin(theta)) / (2.0 * (1.0 - std::cos(theta)))) * w_cross_w_p;
        }

        V_b.tail<3>() = twist;
        
        errors[0] = theta;
        errors[1] = matrix.block<3, 1>(0, 3).norm();

        return V_b;
    }

}
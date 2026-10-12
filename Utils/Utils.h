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
        Eigen::Vector3d w = (theta > 1e-6) ? aa.axis() : Eigen::Vector3d::Zero();

        V_b.head<3>() = theta * w;
        V_b.tail<3>() = p;
        
        errors[0] = theta;
        errors[1] = p.norm();

        return V_b;
    }

}
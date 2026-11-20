#pragma once
#include "eigen3/Eigen/Eigen"
#include "mujoco/mujoco.h"
#include <algorithm>
#include <vector>

namespace Utils {

    inline Eigen::Matrix4d create_HomogenousMatrix(Eigen::Vector3d& pos, Eigen::Matrix3d& rotation_matrix) {

        Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
        result.block<3,3>(0,0) = rotation_matrix;
        result.block<3,1>(0,3) = pos;

        return result;

    }

    inline Eigen::Matrix<double, 6, 1> get_twist_error(double errors[2], Eigen::Matrix4d& matrix) {

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

    inline Eigen::Matrix3d euler_to_rotation(double roll, double pitch, double yaw) {
        Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
        return (rz * ry * rx).toRotationMatrix();
    }

    struct KinematicChain {
        std::vector<int> bodies;        // ordered base -> end effector
        std::vector<int> qpos_indices;  // one per joint in the chain
        std::vector<int> dof_indices;   // one per joint in the chain
    };

    inline KinematicChain get_kinematic_chain(const mjModel* model, int ee_body_id, int base_id = -1) {
        KinematicChain chain;

        if (base_id == -1) {
            base_id = mj_name2id(model, mjOBJ_BODY, "base");
            if (base_id == -1) base_id = 0;
        }

        int current = ee_body_id;
        while (current != base_id && current > 0) {
            chain.bodies.push_back(current);
            current = model->body_parentid[current];
        }
        chain.bodies.push_back(base_id);
        std::reverse(chain.bodies.begin(), chain.bodies.end());

        for (int body_id : chain.bodies) {
            int jnt_id = model->body_jntadr[body_id];
            if (jnt_id != -1) {
                chain.qpos_indices.push_back(model->jnt_qposadr[jnt_id]);
                chain.dof_indices.push_back(model->jnt_dofadr[jnt_id]);
            }
        }

        return chain;
    }

}

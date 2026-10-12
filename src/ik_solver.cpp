#include "ik_solver.hpp"
#include "Utils.h"
#include <iostream>
#include <cmath>

bool Ik_solver::solve(Eigen::Vector3d& target_ee_pos, 
                    Eigen::Matrix3d& target_ee_rotation, 
                    Eigen::VectorXd& initial_guess, 
                    Eigen::VectorXd& solution,
                    double smoothness) {

    constexpr double eps_omega = 1e-3; 
    constexpr double eps_v     = 1e-4; 

    const int num_joints = _qpos_indices.size();
    const int nv = _model->nv;
    const int nq = _model->nq;

    Eigen::VectorXd theta = initial_guess;
    size_t iterations = 50;
    Eigen::Matrix4d T_sd = Utils::create_HomogenousMatrix(target_ee_pos, target_ee_rotation);

    std::vector<mjtNum> jacp(3 * nv);
    std::vector<mjtNum> jacr(3 * nv);

    for (size_t i = 0; i < iterations; ++i) {
        
        mju_copy(_data_ik->qpos, _data->qpos, nq);

        for(int i = 0; i < num_joints; ++i) {
            _data_ik->qpos[_qpos_indices[i]] = theta(i);
        }
        
        mj_fwdPosition(_model, _data_ik);

        Eigen::Vector3d pos(_data_ik->site_xpos[_ee_site_id * 3], 
                            _data_ik->site_xpos[_ee_site_id * 3 + 1], 
                            _data_ik->site_xpos[_ee_site_id * 3 + 2]);
    
        Eigen::Matrix3d rot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            &_data_ik->site_xmat[_ee_site_id * 9]
        );
        Eigen::Matrix4d T_sb = Utils::create_HomogenousMatrix(pos, rot);
    
        Eigen::Matrix4d T_bd =  T_sb.inverse() * T_sd;
    
        double errors[2] = {0, 0};
        Eigen::Matrix<double, 6, 1> Vb = Utils::get_twist_error(errors, T_bd);
    
        if (errors[0] < eps_omega && errors[1] < eps_v) {
            solution = theta;
            std::printf("Found solution ");
            return true;
        }

        mj_jacSite(_model, _data_ik, jacp.data(), jacr.data(), _ee_site_id);
        Eigen::MatrixXd j_pos_world = Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>>(
            jacp.data(), 3, nv
        );
        Eigen::MatrixXd j_rot_world = Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>>(
            jacr.data(), 3, nv
        );
        Eigen::MatrixXd j_rot_body = rot.transpose() * j_rot_world;
        Eigen::MatrixXd j_pos_body = rot.transpose() * j_pos_world;

        Eigen::MatrixXd J_b(6, num_joints);
        for (int i = 0; i < num_joints; ++i) {
            int dof_col = _dof_indices[i];
            J_b.block<3, 1>(0, i) = j_rot_body.col(dof_col);
            J_b.block<3, 1>(3, i) = j_pos_body.col(dof_col);
        }

        Eigen::VectorXd delta_theta = J_b.completeOrthogonalDecomposition().solve(Vb);

        theta += smoothness * delta_theta;
    }

    solution = theta;
    std::printf("can't reach the damn position");
    return false;

}
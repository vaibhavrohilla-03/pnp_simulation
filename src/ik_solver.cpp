#include "ik_solver.hpp"
#include "Utils.h"

#include <cmath>

void Ik_solver::solve(Eigen::Vector3d& target_ee_pos, Eigen::Matrix3d& target_ee_rotation, Eigen::VectorXd& initial_guess) {

    constexpr double eps_omega = 1e-3; 
    constexpr double eps_v     = 1e-4; 

    Eigen::VectorXd theta = initial_guess;
    
    size_t iterations = 50;
    Eigen::Matrix4d T_sd = Utils::create_HomogenousMatrix(target_ee_pos, target_ee_rotation);

    for (size_t i = 0; i < iterations; ++i) {
        
        int nq = _model->nq;
        for (int j = 0; j < theta.size() && j < nq; ++j) {
            _data->qpos[j] = theta(j);
        }
        mj_kinematics(_model, _data);
        
        Eigen::Vector3d pos(_data->site_xpos[_ee_site_id * 3], 
                            _data->site_xpos[_ee_site_id * 3 + 1], 
                            _data->site_xpos[_ee_site_id * 3 + 2]);
    
        Eigen::Matrix3d rot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            &_data->site_xmat[_ee_site_id * 9]
        );
        Eigen::Matrix4d T_sb = Utils::create_HomogenousMatrix(pos, rot);
    
        Eigen::Matrix4d T_bd = T_sd * T_sb.inverse();
    
        double errors[2] = {0, 0};
        Eigen::Matrix<double, 6, 1> Vb = Utils::get_twist_error(errors, T_bd);
    
        if (errors[0] < eps_omega && errors[1] < eps_v) {

            break;
        }


    }

}
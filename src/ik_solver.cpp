#include "ik_solver.hpp"
#include "Utils.h"
#include <cmath>
bool Ik_solver::operator()(const double* target_ee_pos, const double* target_ee_rotation, 
            double* initial_guess, double* solution, const size_t num_joints, double convergence) {
        
        Eigen::AngleAxisd roll(target_ee_rotation[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(target_ee_rotation[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(target_ee_rotation[2], Eigen::Vector3d::UnitZ());
        
        Eigen::Vector3d pos(target_ee_pos[0], target_ee_pos[1], target_ee_pos[2]);
        Eigen::Matrix3d rotation = (yaw * pitch * roll).toRotationMatrix();

        Eigen::VectorXd guess = Eigen::Map<Eigen::VectorXd>(initial_guess, num_joints);
        
        return solve(pos, rotation, guess, solution, convergence);
    }


bool Ik_solver::solve(Eigen::Vector3d& target_ee_pos, 
                    Eigen::Matrix3d& target_ee_rotation, 
                    Eigen::VectorXd& initial_guess, 
                    double* solution,
                    double convergence) {

    constexpr double eps_omega = 1e-3; 
    constexpr double eps_v     = 1e-4; 

    const int num_joints = _qpos_indices.size();
    const int nv = _model->nv;
    const int nq = _model->nq;

    Eigen::VectorXd theta = initial_guess;
    Eigen::Matrix4d T_sd = Utils::create_HomogenousMatrix(target_ee_pos, target_ee_rotation);

    std::vector<mjtNum> jacp(3 * nv);
    std::vector<mjtNum> jacr(3 * nv);

    for (size_t i = 0; i < _iterations; ++i) {
        
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
            Eigen::Map<Eigen::VectorXd>(solution, num_joints) = theta;
            //std::printf("Found solution\n");
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

        theta += convergence * delta_theta;
    }

    Eigen::Map<Eigen::VectorXd>(solution, num_joints) = theta;
    //std::printf("can't reach the damn position\n");
    return false;

}

void Ik_solver::initialize(const std::string& site) {
    
    _base_id = mj_name2id(_model, mjOBJ_BODY, "base");
    if (_base_id == -1) _base_id = 0;
    _ee_site_id = mj_name2id(_model, mjOBJ_SITE, site.c_str());
    _data_ik = mj_makeData(_model);
    
    if(_ee_site_id == -1) {return;}
    
    int ee_body_id = _model->site_bodyid[_ee_site_id];
    int current = ee_body_id;
    while (current != _base_id && current > 0) {
        _kinematic_tree.push_back(current);
        current = _model->body_parentid[current];
    }
    _kinematic_tree.push_back(_base_id);
    std::reverse(_kinematic_tree.begin(), _kinematic_tree.end());
    
    for (int body_id : _kinematic_tree) {
        int jnt_id = _model->body_jntadr[body_id];
        if (jnt_id != -1) {
            _qpos_indices.push_back(_model->jnt_qposadr[jnt_id]);
            _dof_indices.push_back(_model->jnt_dofadr[jnt_id]);
        }
        
    }
        
}

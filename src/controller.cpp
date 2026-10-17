#include "controller.hpp"

void Controller::update(const Eigen::Vector3d& target_pos, const Eigen::Matrix3d& target_rot, Eigen::VectorXd& tau_out) {


    double dt = _model->opt.timestep;
    
    if (_is_first_step) {
        _prev_target_pos = target_pos;
        _prev_target_rot = target_rot;
        _is_first_step = false;
    }
    int nv = _model->nv;
    Eigen::Vector3d current_pos(
        _data->site_xpos[_ee_site_id * 3],
        _data->site_xpos[_ee_site_id * 3 + 1],
        _data->site_xpos[_ee_site_id * 3 + 2]
    );
    
    Eigen::Matrix3d current_rot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        &_data->site_xmat[_ee_site_id * 9]
    );

    Eigen::Vector3d pos_error = target_pos - current_pos;
    Eigen::Matrix3d rot_error_mat = target_rot * current_rot.transpose(); 

    Eigen::Vector3d rot_error = 0.5 * Eigen::Vector3d(
        rot_error_mat(2, 1) - rot_error_mat(1, 2),
        rot_error_mat(0, 2) - rot_error_mat(2, 0),
        rot_error_mat(1, 0) - rot_error_mat(0, 1)
    );

    Eigen::Vector<double, 6> pose_error;
    pose_error.head(3) = pos_error;
    pose_error.tail(3) = rot_error;

    std::vector<mjtNum> jacp(3 * nv);
    std::vector<mjtNum> jacr(3 * nv);
    mj_jacSite(_model, _data, jacp.data(), jacr.data(), _ee_site_id);
    
    Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> Jp(jacp.data(), 3, nv);
    Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> Jr(jacr.data(), 3, nv);
    Eigen::Matrix<double, 6, 6> J;
    
    for (int i = 0; i < _num_joints; ++i) {
        int dof = _dof_indices[i];
        J.col(i).head<3>() = Jp.col(dof);
        J.col(i).tail<3>() = Jr.col(dof);
    }   

    Eigen::VectorXd qdot(_num_joints);

    for (int i = 0; i < _num_joints; ++i) {
        int dof_idx = _dof_indices[i];
        qdot(i) = _data->qvel[dof_idx];
    }

    Eigen::Vector<double, 6> v_current = J * qdot;
    Eigen::Vector<double, 6> v_target = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 6> vel_error = v_target - v_current;

    Eigen::Vector<double, 6> Kp;
    Kp << Kp_pos, Kp_pos, Kp_pos, Kp_rot, Kp_rot, Kp_rot;
    Eigen::Vector<double, 6> Kd;
    Kd << Kd_pos, Kd_pos, Kd_pos, Kd_rot, Kd_rot, Kd_rot;
    Eigen::Vector<double, 6> u_task = Kp.cwiseProduct(pose_error) + Kd.cwiseProduct(vel_error);

    std::vector<mjtNum> M_dense(nv * nv);
    mj_fullM(_model, _data, M_dense.data());
    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> M_full(M_dense.data(), nv, nv);
    Eigen::Matrix<double, 6, 6> M;
    for (int r = 0; r < _num_joints; ++r) {
        for (int c = 0; c < _num_joints; ++c) {
            M(r, c) = M_full(_dof_indices[r], _dof_indices[c]);
        }
    }
    Eigen::Matrix<double, 6, 6> M_inv = M.inverse();
    
    // Lambda = (J * M^-1 * J^T)^-1 with regularization for numerical stability
    Eigen::Matrix<double, 6, 6> Lambda_inv = J * M_inv * J.transpose();
    Eigen::Matrix<double, 6, 6> Mx = (Lambda_inv + 1e-4 * Eigen::Matrix<double, 6, 6>::Identity()).inverse();

    Eigen::VectorXd tau_task = J.transpose() * (Mx * u_task);

    Eigen::Matrix<double, 6, 6> Jbar = M_inv * J.transpose() * Mx;
    Eigen::Matrix<double, 6, 6> null_filter = Eigen::Matrix<double, 6, 6>::Identity() - (J.transpose() * Jbar.transpose());
    Eigen::VectorXd tau_null_desired = -Kd_null * (M * qdot);
    Eigen::VectorXd tau_null = null_filter * tau_null_desired;

    Eigen::VectorXd tau_bias(_num_joints);
    for (int i = 0; i < _num_joints; ++i) {
        tau_bias(i) = _data->qfrc_bias[_dof_indices[i]];
    }

    tau_out = tau_task + tau_null + tau_bias;
    for (int i = 0; i < _num_joints; ++i) {
        tau_out(i) = std::clamp(tau_out(i), -_torque_limits(i), _torque_limits(i));
    }
}

void Controller::operator()(const double* target_ee_pos, const double* target_ee_rotation, double* tau_out) {

    Eigen::AngleAxisd roll(target_ee_rotation[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(target_ee_rotation[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(target_ee_rotation[2], Eigen::Vector3d::UnitZ());

        Eigen::Vector3d pos(target_ee_pos[0], target_ee_pos[1], target_ee_pos[2]);
        Eigen::Matrix3d rotation = (yaw * pitch * roll).toRotationMatrix();

        // 2. Compute OSC Torques
        Eigen::VectorXd tau(_num_joints);
        update(pos, rotation, tau);

        Eigen::Map<Eigen::VectorXd>(tau_out, _num_joints) = tau;
}

void Controller::initialize(const std::string& site) {
    
    _ee_site_id = mj_name2id(_model, mjOBJ_SITE, site.c_str());
    _base_id = mj_name2id(_model, mjOBJ_BODY, "base");
    if (_base_id == -1) _base_id = 0;
    
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
            _dof_indices.push_back(_model->jnt_dofadr[jnt_id]);
        }
    }
        
    _num_joints = _dof_indices.size();
    
    _actuator_indices.clear();
    for (size_t i = 0; i < _num_joints; ++i) {
        int dof = _dof_indices[i];
        int jnt_id = _model->dof_jntid[dof];
    
        // Find the actuator attached to this joint
        int matched_act_id = -1;
        for (int a = 0; a < _model->nu; ++a) {
            if (_model->actuator_trntype[a] == mjTRN_JOINT && 
                _model->actuator_trnid[2 * a] == jnt_id) {
                matched_act_id = a;
                break;
            }
        }
        _actuator_indices.push_back(matched_act_id);
    }

    _torque_limits.resize(_num_joints);
    for (size_t i = 0; i < _num_joints; ++i) {
        int act_id = _actuator_indices[i];
        double limit = 0.0;
        if (act_id >= 0 && act_id < _model->nu) {
            limit = _model->actuator_forcerange[2 * act_id + 1];
        }
        // Fallback to UR5e physical specifications if forcerange is unconstrained in XML
        if (limit <= 0.0) {
            limit = (i < 3) ? 150.0 : 28.0;
        }
        _torque_limits(i) = limit;
    }

    _actuator_backups.clear();
    for (size_t i = 0; i < _num_joints; ++i) {
        int act_id = _actuator_indices[i];
        ActuatorBackup backup;
        if (act_id >= 0 && act_id < _model->nu) {
            backup.biastype = static_cast<mjtBias>(_model->actuator_biastype[act_id]);
            for (int k = 0; k < 10; ++k) {
                backup.gainprm[k] = _model->actuator_gainprm[act_id * 10 + k];
                backup.biasprm[k] = _model->actuator_biasprm[act_id * 10 + k];
            }
            backup.ctrlrange[0] = _model->actuator_ctrlrange[act_id * 2 + 0];
            backup.ctrlrange[1] = _model->actuator_ctrlrange[act_id * 2 + 1];
        }
        _actuator_backups.push_back(backup);
    }

    setTorqueMode(true);
}

void Controller::setTorqueMode(bool enable) {
    for (size_t i = 0; i < _num_joints; ++i) {
        int act_id = _actuator_indices[i];
        if (act_id < 0 || act_id >= _model->nu) continue;

        if (enable) {
            _model->actuator_biastype[act_id] = mjBIAS_NONE;
            for (int k = 0; k < 10; ++k) {
                _model->actuator_gainprm[act_id * 10 + k] = (k == 0) ? 1.0 : 0.0;
                _model->actuator_biasprm[act_id * 10 + k] = 0.0;
            }
            _model->actuator_ctrlrange[act_id * 2 + 0] = -_torque_limits(i);
            _model->actuator_ctrlrange[act_id * 2 + 1] = _torque_limits(i);
        } else {
            if (i < _actuator_backups.size()) {
                _model->actuator_biastype[act_id] = _actuator_backups[i].biastype;
                for (int k = 0; k < 10; ++k) {
                    _model->actuator_gainprm[act_id * 10 + k] = _actuator_backups[i].gainprm[k];
                    _model->actuator_biasprm[act_id * 10 + k] = _actuator_backups[i].biasprm[k];
                }
                _model->actuator_ctrlrange[act_id * 2 + 0] = _actuator_backups[i].ctrlrange[0];
                _model->actuator_ctrlrange[act_id * 2 + 1] = _actuator_backups[i].ctrlrange[1];
            }
        }
    }
}

void Controller::applyTorques(const double* tau) {

    for (size_t i = 0; i < _num_joints; ++i) {
        int act_id = _actuator_indices[i];
        if (act_id >= 0 && act_id < _model->nu) {
            _data->ctrl[act_id] = tau[i];
        }
    }
}

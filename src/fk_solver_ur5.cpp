#include "fk_solver_ur5.hpp"

Eigen::Isometry3d FK_Solver::solve() {

    Eigen::Isometry3d current_chain = Eigen::Isometry3d::Identity();

    for(size_t i = 1; i < kinematic_tree.size(); ++i) {

        int body_id = kinematic_tree[i];

        Eigen::Vector3d position(_model->body_pos + 3 * body_id);
        
        const double* q_ptr = _model->body_quat + 4 * body_id;
        Eigen::Quaterniond quat(q_ptr[0], q_ptr[1], q_ptr[2], q_ptr[3]);
        
        Eigen::Isometry3d T_offset = Eigen::Translation3d(position) * quat;

        int joint_id = _model->body_jntadr[body_id];
        Eigen::Isometry3d T_local;

        if (joint_id != -1) {

             int qpos_adr = _model->jnt_qposadr[joint_id];
             double q_val = _data->qpos[qpos_adr];
            
             Eigen::Vector3d anchor(_model->jnt_pos + 3 * joint_id);
             Eigen::Vector3d axis(_model->jnt_axis + 3 * joint_id);
            
             Eigen::Isometry3d T_anchor = Eigen::Translation3d(anchor) * Eigen::Isometry3d::Identity();
             Eigen::AngleAxisd rot(q_val, axis.normalized());
             Eigen::Isometry3d T_joint = T_anchor * rot * T_anchor.inverse();
            
             T_local = T_offset * T_joint;
            } 
        else {
                T_local = T_offset;
        }
        current_chain = current_chain * T_local;
    }

    return current_chain;    
}


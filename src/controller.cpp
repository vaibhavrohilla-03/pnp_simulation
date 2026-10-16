#include "controller.hpp"

void Controller::update(const Eigen::Vector3d& target_pos, const Eigen::Matrix3d& target_rot, Eigen::VectorXd& tau_out) {

    tau_out.setZero(_num_joints);
    
            int nv = _model->nv;
            Eigen::Vector3d current_pos(
                _data->site_xpos[_ee_site_id * 3],
                _data->site_xpos[_ee_site_id * 3 + 1],
                _data->site_xpos[_ee_site_id * 3 + 2]
            );
    
            Eigen::Matrix3d current_rot = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
                &_data->site_xmat[_ee_site_id * 9]
            );
    
            Eigen::VectorXd q(_num_joints);
            Eigen::VectorXd qdot(_num_joints);
            for (int i = 0; i < _num_joints; ++i) {
                int dof = _dof_indices[i];
                q(i)    = _data->qpos[dof];
                qdot(i) = _data->qvel[dof];
            }
    
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
    
            Eigen::Matrix<double, 6, 1> v_ee = J * qdot;
            Eigen::Vector3d v_pos = v_ee.head<3>();
            Eigen::Vector3d v_rot = v_ee.tail<3>();
    
    
            Eigen::Vector3d e_pos = target_pos - current_pos;
    
            Eigen::Matrix3d R_error = target_rot * current_rot.transpose();
            Eigen::AngleAxisd angle_axis(R_error);
            Eigen::Vector3d e_rot = angle_axis.angle() * angle_axis.axis();
    

            Eigen::Matrix<double, 6, 1> F_cartesian;
            F_cartesian.head<3>() = Kp_pos * e_pos - Kd_pos * v_pos;
            F_cartesian.tail<3>() = Kp_rot * e_rot - Kd_rot * v_rot;
    

            Eigen::VectorXd tau_task = J.transpose() * F_cartesian;

            Eigen::VectorXd tau_gravity(_num_joints);
            for (int i = 0; i < _num_joints; ++i) {
                int dof = _dof_indices[i];
                tau_gravity(i) = _data->qfrc_bias[dof]; 
            }
            
            tau_out = tau_task + tau_gravity;
            
            for (int i = 0; i < _num_joints; ++i) {
                tau_out(i) = std::clamp(tau_out(i), -_torque_limits(i), _torque_limits(i));
            }
}

void Controller::operator()(const double* target_ee_pos, const double* target_ee_rotation, double* tau_out) {

    Eigen::AngleAxisd roll(target_ee_rotation[0], Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch(target_ee_rotation[1], Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw(target_ee_rotation[2], Eigen::Vector3d::UnitZ());

        Eigen::Vector3d pos(target_ee_pos[0], target_ee_pos[1], target_ee_pos[2]);
        Eigen::Matrix3d rotation = (roll * pitch * yaw).toRotationMatrix();

        // 2. Compute OSC Torques
        Eigen::VectorXd tau(_num_joints);
        update(pos, rotation, tau);

        Eigen::Map<Eigen::VectorXd>(tau_out, _num_joints) = tau;


}
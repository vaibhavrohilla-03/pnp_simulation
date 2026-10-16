#include "mujoco/mujoco.h"
#include <vector>
#include <eigen3/Eigen/Eigen>
class Controller {

    public:
        Controller(const std::string& ee_site, const mjModel* model, const mjData* data)
        : _model(model), _data(data)  {

            _ee_site_id = mj_name2id(model, mjOBJ_SITE, ee_site.c_str());
            _base_id = mj_name2id(model, mjOBJ_BODY, "base");
            if (_base_id == -1) _base_id = 0;
            
            int ee_body_id = model->site_bodyid[_ee_site_id];
            int current = ee_body_id;
            while (current != _base_id && current > 0) {
                _kinematic_tree.push_back(current);
                current = model->body_parentid[current];
            }
            _kinematic_tree.push_back(_base_id);
            std::reverse(_kinematic_tree.begin(), _kinematic_tree.end());
            

            for (int body_id : _kinematic_tree) {
                int jnt_id = model->body_jntadr[body_id];
                if (jnt_id != -1) {
                    _dof_indices.push_back(model->jnt_dofadr[jnt_id]);
                }
            }

            _num_joints = _dof_indices.size();
            _q_home.resize(_num_joints);
            
            int keyframe = mj_name2id(model, mjOBJ_KEY, "home");
            mjtNum* keyptr = model->key_qpos + (keyframe * model->nq);

            for (size_t i = 0; i < _num_joints; ++i) {
                int qpos_idx = _dof_indices[i];
                _q_home(i) = keyptr[qpos_idx];
            }

            _torque_limits.resize(_num_joints);
            for (size_t i = 0; i < _num_joints; ++i) {

                int act_id = _dof_indices[i];
                _torque_limits(i) = model->actuator_forcerange[2 * act_id + 1]; 
            }
            
            Kp_pos = 500.0;
            Kd_pos = 40.0;
            Kp_rot = 50.0;
            Kd_rot = 5.0;
            Kp_null = 10.0;
            Kd_null = 2.0;            
    }

        double Kp_pos, Kd_pos;
        double Kp_rot, Kd_rot;
        double Kp_null, Kd_null;

        void operator()(const double* target_ee_pos, 
                    const double* target_ee_rotation, 
                    double* tau_out);

    private:

        void update(const Eigen::Vector3d& target_pos, 
                    const Eigen::Matrix3d& target_rot, 
                    Eigen::VectorXd& tau_out);

        int _ee_site_id;
        int _base_id;

        size_t _num_joints;
        
        std::vector<int> _kinematic_tree;
        std::vector<int> _dof_indices;

        Eigen::VectorXd _q_home;
        Eigen::VectorXd _torque_limits;

        const mjModel* _model;
        const mjData* _data;
    
};
#include "mujoco/mujoco.h"
#include <vector>
#include <eigen3/Eigen/Eigen>
class Controller {

    public:
        Controller(const std::string& ee_site, mjModel* model, const mjData* data)
        : _model(model), _data(data), _is_first_step(true)  {
            
            Kp_pos = 100.0;
            Kd_pos = 20.0;
            Kp_rot = 50.0;
            Kd_rot = 10.0;
            Kp_null = 10.0;
            Kd_null = 2.0;  
                
            initialize(ee_site);
          
    }
        double Kp_pos, Kd_pos;
        double Kp_rot, Kd_rot;
        double Kp_null, Kd_null;

        void operator()(const double* target_ee_pos, 
                    const double* target_ee_rotation, 
                    double* tau_out);

        void applyTorques(const double* tau);
        void setTorqueMode(bool enable);

    private:

        struct ActuatorBackup {
            mjtBias biastype;
            double gainprm[10];
            double biasprm[10];
            double ctrlrange[2];
        };

        void initialize(const std::string& site);
        void update(const Eigen::Vector3d& target_pos, 
                    const Eigen::Matrix3d& target_rot, 
                    Eigen::VectorXd& tau_out);

        int _ee_site_id;
        int _base_id;

        size_t _num_joints;
        
        std::vector<int> _kinematic_tree;
        std::vector<int> _dof_indices;
        std::vector<int> _actuator_indices;
        std::vector<ActuatorBackup> _actuator_backups;
        
        Eigen::VectorXd _torque_limits;

        bool _is_first_step;
        Eigen::Vector3d _prev_target_pos; // <-- Persistent target pos
        Eigen::Matrix3d _prev_target_rot; // <-- Persistent target rot

        mjModel* _model;
        const mjData* _data;
    
};
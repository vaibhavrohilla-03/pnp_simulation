#pragma once
#include <eigen3/Eigen/Eigen>
#include "mujoco/mujoco.h"

class Ik_solver {

    public:
        inline Ik_solver(const std::string& ee_site, const mjModel* model, const mjData* data, const int iterations = 20) : 
             _model(model), _data(data), _iterations(iterations)  {
                initialize(ee_site);
        }   

        bool operator()(const double* target_ee_pos, const double* target_ee_rotation, 
            double* initial_guess, double* solution, const size_t num_joints, double convergence);
            
        inline const int get_num_joints() {return _qpos_indices.size();}

        ~Ik_solver() {
            if (_data_ik) {
                mj_deleteData(_data_ik);
            }
        }


    private:
        int _iterations;
        int _ee_site_id;
        int _base_id;
        
        const mjModel* _model;
        const mjData* _data;
        mjData* _data_ik;

        std::vector<int> _kinematic_tree;
        std::vector<int> _qpos_indices;
        std::vector<int> _dof_indices;
        
        void initialize(const std::string& site);
        
        bool solve(Eigen::Vector3d& target_ee_pos, 
            Eigen::Matrix3d& target_ee_rotation, 
            Eigen::VectorXd& initial_guess, 
            double* solution, 
            double convergence);

        
        
};
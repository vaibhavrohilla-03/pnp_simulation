#pragma once
#include <eigen3/Eigen/Eigen>
#include "mujoco/mujoco.h"
#include "mujoco_raii.hpp"

class Ik_solver {

    public:
        inline Ik_solver(const std::string& ee_site, const mjModel* model, const mjData* data, const int iterations = 20) : 
             _iterations(iterations), _model(model), _data(data)  {
                initialize(ee_site);
        }   

        bool operator()(const double* target_ee_pos, const double* target_ee_rotation, 
            std::vector<double>& initial_guess, std::vector<double>& solution, const size_t num_joints, double step_size);
            
        inline int get_num_joints() const {return _qpos_indices.size();}


    private:
        int _iterations;
        int _ee_site_id;
        int _base_id;
        
        const mjModel* _model;
        const mjData* _data;
        mujoco_raii::DataPtr _data_ik;

        std::vector<int> _kinematic_tree;
        std::vector<int> _qpos_indices;
        std::vector<int> _dof_indices;
        
        void initialize(const std::string& site);
        
        bool solve(Eigen::Vector3d& target_ee_pos, 
            Eigen::Matrix3d& target_ee_rotation, 
            Eigen::VectorXd& initial_guess, 
            double* solution, 
            double step_size);

        
        
};
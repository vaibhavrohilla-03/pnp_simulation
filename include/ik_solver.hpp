#pragma once
#include <eigen3/Eigen/Eigen>
#include "mujoco/mujoco.h"

class Ik_solver {

    public:
        Ik_solver(std::string ee_site, const mjModel* model, const mjData* data) : 
            _ee_site(ee_site), _model(model)  {

                _ee_site_id = mj_name2id(model, mjOBJ_SITE, ee_site.c_str());
                _data = mj_makeData(model);
        }

    private:
        const std::string _ee_site;
        int _ee_site_id;
        
        const mjModel* _model;
        mjData* _data;

        void solve(Eigen::Vector3d& target_ee_pos, Eigen::Matrix3d& target_ee_rotation, Eigen::VectorXd& initial_guess);
        
};
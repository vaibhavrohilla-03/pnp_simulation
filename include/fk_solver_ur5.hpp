#pragma once

#include <eigen3/Eigen/Eigen>
#include "mujoco/mujoco.h"
#include "Utils.hpp"

namespace pnp::fk {

class FK_Solver {
    public:
        inline FK_Solver(const std::string& end_effector, const mjModel* model, const mjData* data) 
            : _model(model), _data(data), _parent_id(0){
            
            _id = mj_name2id(model, mjOBJ_BODY, end_effector.c_str());
            
            if(_parent_id == -1 || _id == -1) {return;}

            if(_parent_id > _id) {return;}

            kinematic_tree = Utils::get_kinematic_chain(model, _id, _parent_id).bodies;

        }
        std::vector<int> kinematic_tree;

        inline Eigen::Vector3d operator()() {
            return solve().translation();
        }

    private:
        const mjModel* _model;
        const mjData_* _data;

        int _parent_id, _id;
        Eigen::Matrix4d Ti;
        
        Eigen::Isometry3d solve();

        
};

}
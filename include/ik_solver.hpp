#pragma once
#include <eigen3/Eigen/Eigen>
#include "mujoco/mujoco.h"

class Ik_solver {

    public:
        Ik_solver(const std::string& base_body, const std::string& ee_site, const mjModel* model, const mjData* data) : 
            _ee_site(ee_site),_base(base_body), _model(model), _data(data)  {

                if(_base == "") {
                    _base_id = 1;
                }
                else {
                    _base_id = mj_name2id(model, mjOBJ_BODY, _base.c_str());
                }

                _ee_site_id = mj_name2id(model, mjOBJ_SITE, ee_site.c_str());
                _data_ik = mj_makeData(model);

            
            if(_base_id == -1 || _ee_site_id == -1) {return;}

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
                    _qpos_indices.push_back(model->jnt_qposadr[jnt_id]);
                    _dof_indices.push_back(model->jnt_dofadr[jnt_id]);
                }

        }
        }   

        bool operator()(const double* target_ee_pos, const double* target_ee_rotation, 
            double* initial_guess, double* solution, const int num_joints, double smoothness);
            
        inline const int get_num_joints() {return _qpos_indices.size();}


        ~Ik_solver() {
            if (_data_ik) {
                mj_deleteData(_data_ik);
            }
        }


    private:
        const std::string _ee_site;
        int _ee_site_id;

        const std::string _base;
        int _base_id;
        
        const mjModel* _model;
        const mjData* _data;
        mjData* _data_ik;

        std::vector<int> _kinematic_tree;
        std::vector<int> _qpos_indices;
        std::vector<int> _dof_indices;
        
        bool solve(Eigen::Vector3d& target_ee_pos, 
            Eigen::Matrix3d& target_ee_rotation, 
            Eigen::VectorXd& initial_guess, 
            double* solution, 
            double smoothness);
        
};
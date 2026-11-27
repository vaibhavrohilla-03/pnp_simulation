#pragma once
#include "app/state.hpp"
#include "mujoco/mujoco.h"
#include <string>
#include <vector>

namespace pnp::app {

    void get_cube_position(const mjModel* model, const mjData* data,
        const std::string& cube_name, std::array<double, 3>& pos);
        
    void build_circle_trajectory(std::vector<double>& timestamps,
        std::vector<double>& waypoints);
            
    void set_arm_control(mjData* data, std::vector<double>& q_target, int num_joints);
        
namespace ui {
    
    void draw_panel(SimState& state, bool solved, double sim_time, double traj_endtime,
        double traj_starttime, const std::function<void()>& reset);

} 
} 

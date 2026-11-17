#pragma once
#include "mujoco/mujoco.h"
#include <string>
#include <vector>

namespace pnp::app {

void get_cube_position(const mjModel* model, const mjData* data,
    const std::string& cube_name, double (&pos)[3]);

void build_circle_trajectory(std::vector<double>& timestamps,
                                std::vector<double>& waypoints);

void set_arm_control(mjData* data, std::vector<double>& q_target, int num_joints);

}

#pragma once

namespace pnp::app {

struct SimState {
    float target_x = 0.0f, target_y = 0.0f, target_z = 0.0f;
    float r_target_x = 0.0f, r_target_y = 0.0f, r_target_z = 0.0f;

    bool use_osc_controller = false;
    bool use_trajectory = false;

    double traj_start_time = -1.0;
};

}

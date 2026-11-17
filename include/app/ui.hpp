#pragma once

#include "app/state.hpp"
#include "controller.hpp"
#include "Trajectory.hpp"
#include "mujoco/mujoco.h"

namespace pnp::app {

void draw_panel(SimState& state, pnp::control::Controller& controller, bool solved,
                double sim_time, const pnp::traj::Trajectory3D& traj,
                mjModel* model, mjData* data);

}

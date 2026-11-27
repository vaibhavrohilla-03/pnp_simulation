#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <span>
#include <vector>

#include "mujoco/mujoco.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "ik_solver.hpp"
#include "osc_controller.hpp"
#include "Trajectory.hpp"
#include "mujoco_raii.hpp"
#include "app/viewer.hpp"
#include "app/state.hpp"
#include "app/helpers.hpp"

using pnp::ik::Ik_solver;
using pnp::control::OscController;
using pnp::traj::Trajectory3D;
using pnp::app::SimState;
using pnp::app::Viewer;

void simulate_loop(mujoco_raii::ModelPtr& model, mujoco_raii::DataPtr& data, Viewer& viewer) {

    Ik_solver solver("gripper_pinch", model.get(), data.get(), 50);
    const size_t num_joints = solver.get_num_joints();
    OscController osc_controller("gripper_pinch", model.get(), data.get());
    std::vector<double> initial_pos(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
        initial_pos[i] = data->qpos[i];
    }

    std::vector<double> solution(num_joints);

    double pos[3];
    pnp::app::get_cube_position(model.get(), data.get(), "my_cube", pos);

    SimState state;
    state.target_x = pos[0];
    state.target_y = pos[1];
    state.target_z = pos[2] - 0.2f;
    state.r_target_x = 3.14159f;
    state.r_target_y = 0.0f;
    state.r_target_z = -1.57079f;
    state.use_osc_controller = true;
    state.use_trajectory = false;

    std::array<double, 3> active_target{state.target_x, state.target_y, state.target_z};
    std::array<double, 3> blend_from = active_target;
    double blend_start = 0.0;
    bool blending = false;
    double blend_duration = 0.5;
    bool prev_use_trajectory = state.use_trajectory;

    const int ee_site = mj_name2id(model.get(), mjOBJ_SITE, "gripper_pinch");
    auto ee_pos = [&data, ee_site]() {
        std::array<double, 3> p;
        for (int i = 0; i < 3; ++i) p[i] = data->site_xpos[ee_site * 3 + i];
        return p;
    };

    state.reset_appstate = [&model, &data, &ee_pos, &active_target, &blend_from, &blending, &blend_start, &blend_duration, &state, &prev_use_trajectory]() {
        mj_resetDataKeyframe(model.get(), data.get(), 0);
        mj_forward(model.get(), data.get());
        blend_from = ee_pos();   // re-anchor commands to the reset pose
        active_target = blend_from;
        blend_start = data->time;
        blend_duration = 1.0;
        blending = true;
        state.traj_start_time = -1.0;
        state.use_trajectory = false;
        prev_use_trajectory = false;
    };

    auto osc_active = [&state]() {
        return state.use_trajectory || (state.use_osc_controller && !state.use_ik);
    };

    state.on_mode_change = [&]() {
        osc_controller.setTorqueMode(osc_active());
    };

    std::vector<double> timestamps;
    std::vector<double> waypoints;
    pnp::app::build_circle_trajectory(timestamps, waypoints);

    Trajectory3D traj;
    if(timestamps.size() < 1 || waypoints.size() != timestamps.size() * 3) {
        std::fprintf(stderr, "null_traj\n");
        return;
    }
    else {
        traj(timestamps, waypoints);
    }

    while(!viewer.should_close()) {

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Generic target-source switch detection: initiates a smooth transition from current commanded target
        if (state.use_trajectory != prev_use_trajectory) {
            blend_from = active_target;   // anchor to current commanded target to guarantee C0 continuity
            blend_start = data->time;
            blending = true;
            if (state.use_trajectory) {
                state.traj_start_time = -1.0; // defer trajectory progression until arrival at entry waypoint
            }

            double dest[3];
            if (state.use_trajectory) {
                traj.evaluate_position(0.0, dest);
            } else {
                dest[0] = state.target_x;
                dest[1] = state.target_y;
                dest[2] = state.target_z;
            }

            double dist = std::sqrt(
                (dest[0] - blend_from[0]) * (dest[0] - blend_from[0]) +
                (dest[1] - blend_from[1]) * (dest[1] - blend_from[1]) +
                (dest[2] - blend_from[2]) * (dest[2] - blend_from[2])
            );
            constexpr double max_blend_vel = 0.6; // m/s
            blend_duration = std::clamp(dist / max_blend_vel, 0.5, 2.5);

            prev_use_trajectory = state.use_trajectory;
        }

        double desired[3];
        double target_rot[3] = {state.r_target_x, state.r_target_y, state.r_target_z};
        std::vector<double> tau_cmd(num_joints);

        // Evaluate target based on active source
        if (state.use_trajectory) {
            if (blending) {
                // While transitioning to trajectory, hold target at spline start point (t = 0)
                traj.evaluate_position(0.0, desired);
            } else {
                if (state.traj_start_time < 0.0) {
                    state.traj_start_time = data->time;
                }
                double elapsed = data->time - state.traj_start_time;
                if (!traj.is_finished(elapsed)) {
                    traj.evaluate_position(elapsed, desired);
                } else {
                    state.traj_start_time = data->time;
                    traj.evaluate_position(0.0, desired);
                }
            }
        } else {
            desired[0] = state.target_x;
            desired[1] = state.target_y;
            desired[2] = state.target_z;
        }

        double blend_t = blending ? std::clamp((data->time - blend_start) / blend_duration, 0.0, 1.0) : 1.0;
        double s = blend_t * blend_t * (3.0 - 2.0 * blend_t);
        double target_pos[3];
        for (int i = 0; i < 3; ++i) {
            target_pos[i] = blending ? blend_from[i] + s * (desired[i] - blend_from[i]) : desired[i];
        }
        if (blend_t >= 1.0) {
            blending = false;
        }
        active_target = {target_pos[0], target_pos[1], target_pos[2]};

        for (size_t i = 0; i < num_joints; ++i) {
            initial_pos[i] = data->qpos[i];
        }
        bool solved = solver(target_pos, target_rot, initial_pos, solution, num_joints, 1.0);

        const bool use_osc = osc_active();
        mjtNum simstart = data->time;
        while(data->time - simstart < 1.0/60.0) {

            if (use_osc) {
                mj_step1(model.get(), data.get());
                osc_controller(target_pos, target_rot, tau_cmd);
                osc_controller.applyTorques(tau_cmd.data());
                mj_step2(model.get(), data.get());
            } else {
                if (solved) {
                    pnp::app::set_arm_control(data.get(), solution, num_joints);
                }
                mj_step(model.get(), data.get());
            }
        }
        pnp::app::ui::draw_panel(state, solved, data->time, traj.end_time(), traj.start_time(), state.reset_appstate);
        viewer.updateframe();

    }
}

void simulate(std::string& file) {

    mujoco_raii::ModelPtr model;
    mujoco_raii::DataPtr data;
    Viewer viewer;

    //mjfLogHandler lg_handler = mju_setLogHandler(error_handler);

    char error[1000];
    model.reset(mj_loadXML(file.c_str(), nullptr, error, sizeof(error)));
    if(!model) {
        std::cerr << "model not found: " << file;
    }
    data.reset(mj_makeData(model.get()));

    mj_resetDataKeyframe(model.get(), data.get(), 0);
    mj_forward(model.get(), data.get());
    viewer.setup(model, data);
    simulate_loop(model, data, viewer);
    //mju_setLogHandler(lg_handler);

    viewer.clean();
    data.reset();
    model.reset();
}

int main() {
    std::string file = std::string(MODEL_DIR) + "/universal_robots_ur5e/scene.xml";
    simulate(file);
    return 0;
}

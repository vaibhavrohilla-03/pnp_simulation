#include <iostream>
#include <memory>
#include <span>
#include <vector>

#include "mujoco/mujoco.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "ik_solver.hpp"
#include "controller.hpp"
#include "Trajectory.hpp"
#include "mujoco_raii.hpp"
#include "app/viewer.hpp"
#include "app/ui.hpp"
#include "app/state.hpp"
#include "app/helpers.hpp"

using pnp::ik::Ik_solver;
using pnp::control::Controller;
using pnp::traj::Trajectory3D;
using pnp::app::SimState;
using pnp::app::Viewer;
using pnp::app::draw_panel;

void simulate_loop(mujoco_raii::ModelPtr& model, mujoco_raii::DataPtr& data, Viewer& viewer) {

    Ik_solver solver("gripper_pinch", model.get(), data.get(), 50);
    const size_t num_joints = solver.get_num_joints();
    Controller controller("gripper_pinch", model.get(), data.get());
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
    state.use_osc_controller = false;
    state.use_trajectory = false;
    controller.setTorqueMode(state.use_osc_controller);

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

         double target_pos[3] = {state.target_x, state.target_y, state.target_z};
         double target_rot[3] = {state.r_target_x, state.r_target_y, state.r_target_z};
         std::vector<double> tau_cmd(num_joints);

        // If trajectory mode is active, override target_pos from spline
        if (state.use_trajectory) {
        if (state.traj_start_time < 0.0) {
            state.traj_start_time = data->time;   // record when we started
        }
            double elapsed = data->time - state.traj_start_time;
            if (!traj.is_finished(elapsed)) {
                double traj_pos[3];
                traj.evaluate_position(elapsed, traj_pos);
                target_pos[0] = traj_pos[0];
                target_pos[1] = traj_pos[1];
                target_pos[2] = traj_pos[2];
            } else {

                state.traj_start_time = data->time;
            }
        }
        for (size_t i = 0; i < num_joints; ++i) {
            initial_pos[i] = data->qpos[i];
        }
        bool solved = solver(target_pos, target_rot, initial_pos, solution, num_joints, 1.0);

        mjtNum simstart = data->time;
        while(data->time - simstart < 1.0/60.0) {

            if (state.use_osc_controller) {
                mj_step1(model.get(), data.get());

                if(solved) {
                    controller(target_pos, target_rot, tau_cmd);
                    controller.applyTorques(tau_cmd.data());
                }

                mj_step2(model.get(), data.get());
            } else {
                if (solved) {
                    pnp::app::set_arm_control(data.get(), solution, num_joints);
                }
                mj_step(model.get(), data.get());
            }
        }
        draw_panel(state, controller, solved, data->time, traj, model.get(), data.get());
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

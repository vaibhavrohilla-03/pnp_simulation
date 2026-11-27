#include "app/helpers.hpp"
#include "imgui.h"
#include <cmath>

namespace pnp::app {

void get_cube_position(const mjModel* model, const mjData* data,
    const std::string& cube_name, std::array<double, 3>& pos) {

    int body_id = mj_name2id(model, mjOBJ_BODY, cube_name.c_str());

    pos[0] = data->xpos[3 * body_id];
    pos[1] = data->xpos[3 * body_id + 1];
    pos[2] = data->xpos[3 * body_id + 2];
}

void build_circle_trajectory(std::vector<double>& timestamps,
                                std::vector<double>& waypoints) {
     double center_x = 0.4;
    double center_y = 0.0;
    double center_z = 0.4;
    double radius   = 0.15;
    double total_time = 6.0;
    int num_circle_points = 24;

    for (int i = 0; i <= num_circle_points; ++i) {
        double t = total_time * i / num_circle_points;
        double angle = 2.0 * M_PI * i / num_circle_points;

        timestamps.push_back(t);
        waypoints.push_back(center_x + radius * cos(angle));
        waypoints.push_back(center_y + radius * sin(angle));
        waypoints.push_back(center_z);
    }
}

void set_arm_control(mjData* data, std::vector<double>& q_target, int num_joints) {

    for (int i = 0; i < num_joints; ++i) {
        data->ctrl[i] = q_target[i];
    }
}

namespace ui {

void draw_panel(SimState& state, bool solved, double sim_time, double traj_endtime, double traj_starttime, const std::function<void()>& reset) {

    ImGui::Begin("Controller & IK");
    
    bool traj = state.use_trajectory;
    if (!state.use_trajectory) {
        
        bool osc = state.use_osc_controller;
        if (ImGui::Checkbox("Use OSC Controller (Torque)", &osc)) {
            state.use_osc_controller = osc;
            state.use_ik = !osc;
            if (state.on_mode_change) state.on_mode_change();
        }
        
        bool ik = state.use_ik;
        if (ImGui::Checkbox("Use IK (position control)", &ik)) {
            state.use_ik = ik;
            state.use_osc_controller = !ik;
            if (state.on_mode_change) state.on_mode_change();
        }   
    }
    
    if (ImGui::Checkbox("Run Circle Trajectory", &traj)) {
        state.use_trajectory = traj;
        if (state.on_mode_change) state.on_mode_change();
    }
    
    ImGui::Separator();

    if (solved) {
        ImGui::Text("IK Status: Solved");
    } else if (state.use_trajectory || (state.use_osc_controller && !state.use_ik)) {
        ImGui::Separator();
        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f),
            "IK: Unreachable - OSC still driving toward target");
    } else {
        ImGui::Text("IK Status: Unreachable / Iterating");
    }
    if (!state.use_trajectory) {
        ImGui::SliderFloat("x", &state.target_x, -1.0f, 1.0f);
        ImGui::SliderFloat("y", &state.target_y, -1.0f, 1.0f);
        ImGui::SliderFloat("z", &state.target_z, -1.0f, 1.0f);
    } else {
        ImGui::Text("Trajectory Time: %.2f / %.2f s",
            state.traj_start_time >= 0.0 ? sim_time - state.traj_start_time : 0.0,
            traj_endtime - traj_starttime);
    }
    ImGui::SliderFloat("rot_x", &state.r_target_x, -3.14f, +3.14f);
    ImGui::SliderFloat("rot_y", &state.r_target_y, -3.14f, +3.14f);
    ImGui::SliderFloat("rot_z", &state.r_target_z, -3.14f, +3.14f);
    ImGui::Text("Target Pos: (%.2f, %.2f, %.2f)", state.target_x, state.target_y, state.target_z);
    if (ImGui::Button("Reset Scene")) {
        reset();
        state.traj_start_time = -1.0;
    }
    ImGui::End();
}

}
}
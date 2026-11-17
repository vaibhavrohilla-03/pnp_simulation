#include "app/ui.hpp"
#include "imgui.h"

namespace pnp::app {

void draw_panel(SimState& state, pnp::control::Controller& controller, bool solved,
                double sim_time, const pnp::traj::Trajectory3D& traj,
                mjModel* model, mjData* data) {

    ImGui::Begin("Controller & IK");
        if (ImGui::Checkbox("Use OSC Controller (Torque)", &state.use_osc_controller)) {
            controller.setTorqueMode(state.use_osc_controller);
        }
        ImGui::Checkbox("Run Circle Trajectory", &state.use_trajectory);
        if (!state.use_trajectory) {
            state.traj_start_time = -1.0;  // reset so it starts fresh next time
        }
        ImGui::Separator();
        ImGui::Text("IK Status: %s", solved ? "Solved" : "Unreachable / Iterating");
        if (!state.use_trajectory) {
            ImGui::SliderFloat("x", &state.target_x, -1.0f, 1.0f);
            ImGui::SliderFloat("y", &state.target_y, -1.0f, 1.0f);
            ImGui::SliderFloat("z", &state.target_z, -1.0f, 1.0f);
        } else {
            ImGui::Text("Trajectory Time: %.2f / %.2f s",
                state.traj_start_time >= 0.0 ? sim_time - state.traj_start_time : 0.0,
                traj.end_time() - traj.start_time());
        }
        ImGui::SliderFloat("rot_x", &state.r_target_x, -3.14f, +3.14f);
        ImGui::SliderFloat("rot_y", &state.r_target_y, -3.14f, +3.14f);
        ImGui::SliderFloat("rot_z", &state.r_target_z, -3.14f, +3.14f);

        ImGui::Text("Target Pos: (%.2f, %.2f, %.2f)", state.target_x, state.target_y, state.target_z);

        if (ImGui::Button("Reset Scene")) {
            mj_resetDataKeyframe(model, data, 0);
            mj_forward(model, data);
            state.traj_start_time = -1.0;
        }
        ImGui::End();
}

}

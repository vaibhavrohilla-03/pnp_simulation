#include "app/helpers.hpp"
#include <cmath>

namespace pnp::app {

void get_cube_position(const mjModel* model, const mjData* data,
    const std::string& cube_name, double (&pos)[3]) {

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

}

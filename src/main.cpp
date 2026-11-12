#include <iostream>
#include <cmath>
#include <memory>

#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

// #include "fk_solver_ur5.hpp"
#include "ik_solver.hpp"
#include "controller.hpp"
#include "Trajectory.hpp"
#include "mujoco_raii.hpp"
#include <span>
struct GlfwWindowDeleter {
    void operator()(GLFWwindow* window) const noexcept {
        if (window != nullptr) {
            glfwDestroyWindow(window);
        }
    }
};

using GlfwWindowPtr = std::unique_ptr<GLFWwindow, GlfwWindowDeleter>;

struct visual {

mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GlfwWindowPtr window;

bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
};

mujoco_raii::ModelPtr model;
mujoco_raii::DataPtr data;

struct visual visualdata;

mjtNum position_history = 0;
mjtNum previous_time = 0;

float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

int joint_idx = 0;


// void scratchpad(mjModel* model) {
//     return;
// }

void error_handler(const mjLogMessage* msg) {

    std::printf("%s \n", msg->subject);

}

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{       
    if(ImGui::GetIO().WantCaptureKeyboard) return;

    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(model.get(), data.get());
        mj_forward(model.get(), data.get());
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods)
{   
    if(ImGui::GetIO().WantCaptureMouse) return;
    visualdata.button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    visualdata.button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    visualdata.button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    glfwGetCursorPos(window, &visualdata.lastx, &visualdata.lasty);
}


void mouse_move(GLFWwindow* window, double xpos, double ypos)
{   
    if(ImGui::GetIO().WantCaptureMouse) return;
    if( !visualdata.button_left && !visualdata.button_middle && !visualdata.button_right )
        return;

    double dx = xpos - visualdata.lastx;
    double dy = ypos - visualdata.lasty;
    visualdata.lastx = xpos;
    visualdata.lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    mjtMouse action;
    if( visualdata.button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( visualdata.button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(model.get(), action, dx/height, dy/height, &visualdata.cam);
}


void scroll(GLFWwindow* window, double xoffset, double yoffset)
{   
    if(ImGui::GetIO().WantCaptureMouse) return;
    mjv_moveCamera(model.get(), mjMOUSE_ZOOM, 0, -0.05*yoffset, &visualdata.cam);
}
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

void setup_render() {

    if(!glfwInit()) {
        std::cerr << "Could not initialize GLFW";
    }

    visualdata.window.reset(glfwCreateWindow(1244, 700, "simulation", NULL, NULL));

    glfwMakeContextCurrent(visualdata.window.get());
    glfwSwapInterval(1);

    mjv_defaultCamera(&visualdata.cam);
    mjv_defaultOption(&visualdata.opt);

    mjv_defaultScene(&visualdata.scn);
    
    // visualdata.scn.flags[mjRND_REFLECTION] = 0; 
    // visualdata.scn.flags[mjRND_SHADOW] = 0;
    // visualdata.scn.flags[mjRND_SKYBOX] = 0;

    mjr_defaultContext(&visualdata.con);
    mjv_makeScene(model.get(), &visualdata.scn, 2000);
    mjr_makeContext(model.get(), &visualdata.con, mjFONTSCALE_150);

    glfwSetKeyCallback(visualdata.window.get(), keyboard);
    glfwSetCursorPosCallback(visualdata.window.get(), mouse_move);
    glfwSetMouseButtonCallback(visualdata.window.get(), mouse_button);
    glfwSetScrollCallback(visualdata.window.get(), scroll);

    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    
    ImGui_ImplGlfw_InitForOpenGL(visualdata.window.get(), true);
    ImGui_ImplOpenGL3_Init();

}

void updateframe() {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(visualdata.window.get(), &viewport.width, &viewport.height);
        
    //visualdata.opt.frame = mjFRAME_BODY
    //std::printf("%lf \n", model->opt.timestep);
    visualdata.opt.flags[2] = mjVIS_JOINT;
    

    mjv_updateScene(model.get(), data.get(), &visualdata.opt, NULL, &visualdata.cam, mjCAT_ALL, &visualdata.scn);
    mjr_render(viewport, &visualdata.scn, &visualdata.con);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    
    glfwSwapBuffers(visualdata.window.get());    
    glfwPollEvents();
}

void simulate_loop() {
    
    Ik_solver solver("gripper_pinch", model.get(), data.get(), 50); 
    const size_t num_joints = solver.get_num_joints(); 
    Controller controller("gripper_pinch", model.get(), data.get());
    std::vector<double> initial_pos(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
        initial_pos[i] = data->qpos[i];
    }
    
    std::vector<double> solution(num_joints);
    
    double pos[3];
    get_cube_position(model.get(), data.get(), "my_cube", pos);
    
    static float target_x = pos[0];
    static float target_y = pos[1];
    static float target_z = pos[2] - 0.2f;
    static float r_target_x = 3.14159f;
    static float r_target_y = 0.0f;
    static float r_target_z = -1.57079f;
    
    static bool use_osc_controller = false;
    static bool use_trajectory = false;
    controller.setTorqueMode(use_osc_controller);

    std::vector<double> timestamps;
    std::vector<double> waypoints;  
    build_circle_trajectory(timestamps, waypoints);
    
    Trajectory3D traj;
    if(timestamps.size() < 1 || waypoints.size() != timestamps.size() * 3) {
        std::fprintf(stderr, "null_traj\n");
        return;
    }
    else {
        traj(timestamps, waypoints);
    }
    double traj_start_time = -1.0;

    while(!glfwWindowShouldClose(visualdata.window.get())) {
        
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

         double target_pos[3] = {target_x, target_y, target_z};
         double target_rot[3] = {r_target_x, r_target_y, r_target_z};
         std::vector<double> tau_cmd(num_joints);

        // If trajectory mode is active, override target_pos from spline
        if (use_trajectory) {
        if (traj_start_time < 0.0) {
            traj_start_time = data->time;   // record when we started
        }
            double elapsed = data->time - traj_start_time;
            if (!traj.is_finished(elapsed)) {
                double traj_pos[3];
                traj.evaluate_position(elapsed, traj_pos);
                target_pos[0] = traj_pos[0];
                target_pos[1] = traj_pos[1];
                target_pos[2] = traj_pos[2];
            } else {

                traj_start_time = data->time;
            }
        }
        for (size_t i = 0; i < num_joints; ++i) {
            initial_pos[i] = data->qpos[i];
        }
        bool solved = solver(target_pos, target_rot, initial_pos, solution, num_joints, 1.0);

        mjtNum simstart = data->time;
        while(data->time - simstart < 1.0/60.0) {

            if (use_osc_controller) {
                mj_step1(model.get(), data.get());

                if(solved) {
                    controller(target_pos, target_rot, tau_cmd);
                    controller.applyTorques(tau_cmd.data());
                }

                mj_step2(model.get(), data.get());
            } else {
                if (solved) {
                    set_arm_control(data.get(), solution, num_joints);
                }
                mj_step(model.get(), data.get());
            }
        }
        ImGui::Begin("Controller & IK");
            if (ImGui::Checkbox("Use OSC Controller (Torque)", &use_osc_controller)) {
                controller.setTorqueMode(use_osc_controller);
            }
            ImGui::Checkbox("Run Circle Trajectory", &use_trajectory);
            if (!use_trajectory) {
                traj_start_time = -1.0;  // reset so it starts fresh next time
            }
            ImGui::Separator();
            ImGui::Text("IK Status: %s", solved ? "Solved" : "Unreachable / Iterating");
            if (!use_trajectory) {
                ImGui::SliderFloat("x", &target_x, -1.0f, 1.0f);
                ImGui::SliderFloat("y", &target_y, -1.0f, 1.0f);
                ImGui::SliderFloat("z", &target_z, -1.0f, 1.0f);
            } else {
                ImGui::Text("Trajectory Time: %.2f / %.2f s", 
                    traj_start_time >= 0.0 ? data->time - traj_start_time : 0.0,
                    traj.end_time() - traj.start_time());
            }
            ImGui::SliderFloat("rot_x", &r_target_x, -3.14f, +3.14f);
            ImGui::SliderFloat("rot_y", &r_target_y, -3.14f, +3.14f);
            ImGui::SliderFloat("rot_z", &r_target_z, -3.14f, +3.14f);

            ImGui::Text("Target Pos: (%.2f, %.2f, %.2f)", target_pos[0], target_pos[1], target_pos[2]);

            if (ImGui::Button("Reset Scene")) {
                mj_resetDataKeyframe(model.get(), data.get(), 0);
                mj_forward(model.get(), data.get());
                traj_start_time = -1.0;
            }
            ImGui::End();
        updateframe();
        
    }
}

void clean() {

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    mjv_freeScene(&visualdata.scn);
    mjr_freeContext(&visualdata.con);
    visualdata.window.reset();
    glfwTerminate();
    data.reset();
    model.reset();
}

void simulate(std::string& file) {

    //mjfLogHandler lg_handler = mju_setLogHandler(error_handler);
    
    char error[1000];
    model.reset(mj_loadXML(file.c_str(), nullptr, error, sizeof(error)));
    if(!model) {
        std::cerr << "model not found: " << file;
    }
    data.reset(mj_makeData(model.get()));

    mj_resetDataKeyframe(model.get(), data.get(), 0);
    mj_forward(model.get(), data.get());
    setup_render();
    simulate_loop();
    //mju_setLogHandler(lg_handler);
}

int main() {
    std::string file = std::string(MODEL_DIR) + "/universal_robots_ur5e/scene.xml";
    simulate(file); 
    clean();   
    return 0;
}
#include <iostream>
#include <cmath>

#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "fk_solver_ur5.hpp"

struct visual {

mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window = nullptr;

bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
};

mjModel* model = nullptr;
mjData* data = nullptr;

struct visual visualdata;

mjtNum position_history = 0;
mjtNum previous_time = 0;

float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

int joint_idx = 0;

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(model, data);
        mj_forward(model, data);
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    visualdata.button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    visualdata.button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    visualdata.button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    glfwGetCursorPos(window, &visualdata.lastx, &visualdata.lasty);
}


void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
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

    mjv_moveCamera(model, action, dx/height, dy/height, &visualdata.cam);
}


void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &visualdata.cam);
}

void applydragforce_cube(double drag_coefficient, bool cached) {

    if(!cached) {
        int joint_id = mj_name2id(model, mjOBJ_JOINT, "slide_x");
        joint_idx = model->jnt_dofadr[joint_id];
        cached = true;
    }

    double vx, vy, vz = 0.0;
    
    vx = data->qvel[joint_idx];
    vy = data->qvel[joint_idx + 1];
    vz = data->qvel[joint_idx + 2];

    double v = std::sqrt(vx*vx + vy*vy + vz*vz);


    data->qfrc_applied[joint_idx] = -drag_coefficient * v * vx;
    data->qfrc_applied[joint_idx + 1] = -drag_coefficient * v * vy;
    data->qfrc_applied[joint_idx + 2] = -drag_coefficient * v * vz;

}

void setup_render() {

    if(!glfwInit()) {
        std::cerr << "Could not initialize GLFW";
    }

    visualdata.window = glfwCreateWindow(1244, 700, "simulation", NULL, NULL);

    glfwMakeContextCurrent(visualdata.window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&visualdata.cam);
    mjv_defaultOption(&visualdata.opt);

    visualdata.scn.flags[mjRND_REFLECTION] = 1; 
    visualdata.scn.flags[mjRND_SHADOW] = 1;
    visualdata.scn.flags[mjRND_SKYBOX] = 1;

    mjv_defaultScene(&visualdata.scn);
    mjr_defaultContext(&visualdata.con);
    mjv_makeScene(model, &visualdata.scn, 2000);
    mjr_makeContext(model, &visualdata.con, mjFONTSCALE_150);

    glfwSetKeyCallback(visualdata.window, keyboard);
    glfwSetCursorPosCallback(visualdata.window, mouse_move);
    glfwSetMouseButtonCallback(visualdata.window, mouse_button);
    glfwSetScrollCallback(visualdata.window, scroll);
}

void updateframe() {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(visualdata.window, &viewport.width, &viewport.height);
        
    visualdata.opt.frame = mjFRAME_BODY;
    mjv_updateScene(model, data, &visualdata.opt, NULL, &visualdata.cam, mjCAT_ALL, &visualdata.scn);
    mjr_render(viewport, &visualdata.scn, &visualdata.con);
    
    glfwSwapBuffers(visualdata.window);
        
    glfwPollEvents();
}

void simulate_loop() {

    bool cache = false;
    FK_Solver solver("wrist_3_link", model, data);
    int ee_id = mj_name2id(model, mjOBJ_BODY, "wrist_3_link");

    int frame = 0;
    while(!glfwWindowShouldClose(visualdata.window)) {

        mjtNum simstart = data->time;

        while(data->time - simstart < 1.0/60.0) {
            
            applydragforce_cube(10, cache);
            mj_step(model, data);    
        }
        if(++frame % 30 == 0) {
            Eigen::Vector3d my_pos = solver();               
            Eigen::Vector3d mj_pos(data->xpos + 3 * ee_id);
            
            std::printf("from solver : %lf, %lf, %lf\n", my_pos.x(), my_pos.y(), my_pos.z());
            std::printf("from mj_solver : %lf, %lf, %lf\n", mj_pos.x(), mj_pos.y(), mj_pos.z());
            std::cout << "Error: " << (my_pos - mj_pos).norm() << " m\n";
        }
        updateframe();
        
    }
}

void clean() {
    mjv_freeScene(&visualdata.scn);
    mjr_freeContext(&visualdata.con);
    glfwDestroyWindow(visualdata.window);
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
}



void simulate(std::string& file) {

    char error[1000];
    model = mj_loadXML(file.c_str(), nullptr, error, sizeof(error));
    if(!model) {
        std::cerr << "model not found: " << file;
    }
    data = mj_makeData(model);

    setup_render();
    simulate_loop();
}

int main(int argc, char** argv) {

    std::string file = std::string(MODEL_DIR) + "/universal_robots_ur5e/ur5e_with_gripper.xml";
    simulate(file); 
    clean();   
    return 0;
}
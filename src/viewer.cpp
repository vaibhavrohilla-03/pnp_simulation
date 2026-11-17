#include "app/viewer.hpp"
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <iostream>

namespace pnp::app {

void Viewer::keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    if(ImGui::GetIO().WantCaptureKeyboard) return;

    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        auto* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
        mj_resetData(self->_model, self->_data);
        mj_forward(self->_model, self->_data);
    }
}

void Viewer::mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    if(ImGui::GetIO().WantCaptureMouse) return;
    auto* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));

    self->_button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    self->_button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    self->_button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    glfwGetCursorPos(window, &self->_lastx, &self->_lasty);
}


void Viewer::mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    if(ImGui::GetIO().WantCaptureMouse) return;
    auto* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    if( !self->_button_left && !self->_button_middle && !self->_button_right )
        return;

    double dx = xpos - self->_lastx;
    double dy = ypos - self->_lasty;
    self->_lastx = xpos;
    self->_lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    mjtMouse action;
    if( self->_button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( self->_button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    mjv_moveCamera(self->_model, action, dx/height, dy/height, &self->_cam);
}


void Viewer::scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    if(ImGui::GetIO().WantCaptureMouse) return;
    auto* self = static_cast<Viewer*>(glfwGetWindowUserPointer(window));
    mjv_moveCamera(self->_model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &self->_cam);
}

void Viewer::setup(mujoco_raii::ModelPtr& model, mujoco_raii::DataPtr& data) {

    _model = model.get();
    _data = data.get();

    if(!glfwInit()) {
        std::cerr << "Could not initialize GLFW";
    }

    _window.reset(glfwCreateWindow(1244, 700, "simulation", NULL, NULL));

    glfwMakeContextCurrent(_window.get());
    glfwSwapInterval(1);

    mjv_defaultCamera(&_cam);
    mjv_defaultOption(&_opt);

    mjv_defaultScene(&_scn);

    // _scn.flags[mjRND_REFLECTION] = 0;
    // _scn.flags[mjRND_SHADOW] = 0;
    // _scn.flags[mjRND_SKYBOX] = 0;

    mjr_defaultContext(&_con);
    mjv_makeScene(_model, &_scn, 2000);
    mjr_makeContext(_model, &_con, mjFONTSCALE_150);

    glfwSetWindowUserPointer(_window.get(), this);
    glfwSetKeyCallback(_window.get(), keyboard);
    glfwSetCursorPosCallback(_window.get(), mouse_move);
    glfwSetMouseButtonCallback(_window.get(), mouse_button);
    glfwSetScrollCallback(_window.get(), scroll);

    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(_window.get(), true);
    ImGui_ImplOpenGL3_Init();

}

void Viewer::updateframe() {
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(_window.get(), &viewport.width, &viewport.height);

    //_opt.frame = mjFRAME_BODY
    //std::printf("%lf \n", _model->opt.timestep);
    _opt.flags[2] = mjVIS_JOINT;


    mjv_updateScene(_model, _data, &_opt, NULL, &_cam, mjCAT_ALL, &_scn);
    mjr_render(viewport, &_scn, &_con);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(_window.get());
    glfwPollEvents();
}

bool Viewer::should_close() const {
    return glfwWindowShouldClose(_window.get());
}

void Viewer::clean() {

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    mjv_freeScene(&_scn);
    mjr_freeContext(&_con);
    _window.reset();
    glfwTerminate();
}

}

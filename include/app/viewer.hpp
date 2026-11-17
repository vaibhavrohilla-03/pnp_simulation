#pragma once

#include "mujoco_raii.hpp"
#include "GLFW/glfw3.h"
#include <memory>

namespace pnp::app {

class Viewer {
public:
    void setup(mujoco_raii::ModelPtr& model, mujoco_raii::DataPtr& data);
    void updateframe();
    bool should_close() const;
    void clean();

private:
    struct GlfwWindowDeleter {
        void operator()(GLFWwindow* window) const noexcept {
            if (window != nullptr) {
                glfwDestroyWindow(window);
            }
        }
    };

    using GlfwWindowPtr = std::unique_ptr<GLFWwindow, GlfwWindowDeleter>;

    static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
    static void mouse_button(GLFWwindow* window, int button, int act, int mods);
    static void mouse_move(GLFWwindow* window, double xpos, double ypos);
    static void scroll(GLFWwindow* window, double xoffset, double yoffset);

    mjModel* _model = nullptr;
    mjData* _data = nullptr;

    GlfwWindowPtr _window;

    mjvCamera _cam;
    mjvOption _opt;
    mjvScene _scn;
    mjrContext _con;

    bool _button_left = false;
    bool _button_middle = false;
    bool _button_right = false;
    double _lastx = 0;
    double _lasty = 0;
};

}

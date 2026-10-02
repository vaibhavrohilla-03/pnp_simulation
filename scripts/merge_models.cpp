#include "mujoco/mujoco.h"
#include <iostream>
int main() {

    char error[1000] ="";
    std::string model_dir(MODEL_DIR);
    
    std::string ur5_file = model_dir + "/universal_robots_ur5e/ur5e.xml";
    auto spec_ur5 = mj_parseXML(ur5_file.c_str(), nullptr, error, sizeof(error)); 

    std::string gripper_file = model_dir + "/robotiq_2f85/2f85.xml";
    auto spec_gripper = mj_parseXML(gripper_file.c_str(), nullptr, error, sizeof(error));

    mjsElement* site_element = mjs_findElement(spec_ur5, mjOBJ_SITE, "attachment_site");
    
    mjs_attach(site_element, spec_gripper->element, "gripper_", "");

    std::string save = model_dir + "/universal_robots_ur5e/ur5e_with_gripper.xml";
    
    mjModel* model = mj_compile(spec_ur5, nullptr);
    if (!model) {
        std::cerr << "Failed to compile merged model." << std::endl;
        return 1;
    }

    if (mj_saveXML(spec_ur5, save.c_str(), error, sizeof(error)) != 1) {
        std::cerr  << error << "\n";
    } 
    else {
        std::cout << "Successfully saved to: " << save << "\n";
    }

    mj_deleteModel(model);
    mj_deleteSpec(spec_ur5);

    return 0;
}
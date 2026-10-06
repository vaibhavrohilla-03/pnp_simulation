cmake_minimum_required(VERSION "3.20")
project(pnp_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS /opt/mujoco/include)
find_library(MUJOCO_LIBRARY  mujoco PATHS /opt/mujoco/lib)


add_subdirectory(thirdparty)

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
)

add_executable(playground src/main.cpp)

target_include_directories(playground PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_sources(playground PRIVATE
    src/fk_solver_ur5.cpp
)


target_link_libraries(playground PRIVATE
    glfw
    mujoco::mujoco
    Eigen3::Eigen)
target_compile_definitions(playground PRIVATE
     MODEL_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/models\"
)
set_target_properties(playground PROPERTIES
    BUILD_RPATH "/opt/mujoco/lib"
    INSTALL_RPATH "/opt/mujoco/lib"
)


add_executable(merge scripts/merge_models.cpp)
target_include_directories(merge PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
target_link_libraries(merge PRIVATE 
    mujoco::mujoco)
target_compile_definitions(merge PRIVATE
    MODEL_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/models\"
)
set_target_properties(merge PROPERTIES
    BUILD_RPATH "/opt/mujoco/lib"
    INSTALL_RPATH "/opt/mujoco/lib"
)

add_custom_command(
    TARGET merge 
    POST_BUILD
    COMMAND $<TARGET_FILE:merge>
    COMMENT "Running merge script to generate ur5e_with_gripper.xml"
)

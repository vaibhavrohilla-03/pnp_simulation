set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(WANT_DOCS OFF CACHE BOOL "" FORCE)
set(WANT_TESTS OFF CACHE INTERNAL "")
set(WANT_EXAMPLES OFF CACHE INTERNAL "")
set(WANT_DEMO OFF CACHE INTERNAL "")
set(SHARED OFF CACHE INTERNAL "")
set(WANT_PRIMITIVES ON CACHE INTERNAL "")
set(WANT_IMAGE ON CACHE INTERNAL "")
set(WANT_FONT ON CACHE INTERNAL "")
set(WANT_TTF ON CACHE INTERNAL "")
    
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/glfw/CMakeLists.txt")
    add_subdirectory(thirdparty/glfw)
else()
    include(FetchContent)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(glfw)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen/CMakeLists.txt")
    add_subdirectory(thirdparty/eigen)
else()
    FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0
    GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(eigen)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/CMakeLists.txt")
    add_subdirectory(thirdparty/imgui)
else()

    FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
    GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(imgui)

    add_library(imgui
	${imgui_SOURCE_DIR}/imgui.cpp
	${imgui_SOURCE_DIR}/imgui_draw.cpp
	${imgui_SOURCE_DIR}/imgui_tables.cpp
	${imgui_SOURCE_DIR}/imgui_widgets.cpp
	${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
	${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp

)
target_include_directories(imgui PUBLIC 
${imgui_SOURCE_DIR}
${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC 
    glfw
)
endif()
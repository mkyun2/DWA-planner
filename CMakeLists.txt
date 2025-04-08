cmake_minimum_required(VERSION 3.10)
project(mobile_robot_project)

# C++ 표준 설정 (예: C++17)
set(TARGET simulate)
set(CMAKE_CXX_STANDARD 17)
set(OUTPUT_DIR ${PROJECT_SOURCE_DIR}/output)
add_library(my_robot_lib
    src/robot.cpp
    src/motion_planner.cpp
    src/motion_controller.cpp
)

# 헤더 디렉터리 (라이브러리 헤더를 외부에서 쓰려면 PUBLIC)
target_include_directories(my_robot_lib
    PUBLIC ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# main을 빌드할 때 라이브러리 링크
add_executable(${TARGET} main.cpp)
target_link_libraries(${TARGET} PUBLIC my_robot_lib)
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
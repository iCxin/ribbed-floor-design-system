cmake_minimum_required(VERSION 3.12)
project(ribbed_floor_design_system CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ribbed_floor_design_system
    src/main.cpp
    src/protocol.cpp
    src/hal_mock.cpp
    src/param_parser.cpp
    src/struct_calc.cpp
    src/rebar_optim.cpp
    src/result_pack.cpp
    src/data_storage.cpp
    src/system_controller.cpp
)

target_include_directories(ribbed_floor_design_system PRIVATE include)

if(WIN32)
    target_link_libraries(ribbed_floor_design_system PRIVATE ws2_32)
endif()

if(MSVC)
    target_compile_options(ribbed_floor_design_system PRIVATE /W4 /utf-8)
else()
    target_compile_options(ribbed_floor_design_system PRIVATE -Wall -Wextra)
endif()

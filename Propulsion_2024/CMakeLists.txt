cmake_minimum_required(VERSION 3.28)
project(Propulsion_2024)

set(CMAKE_CXX_STANDARD 14)
add_compile_options("-DMOCK_RPI")

include_directories(.
        lib)

add_executable(Propulsion_2024
        lib/Command.h
        lib/Command_Interpreter.cpp
        lib/Command_Interpreter.h
        lib/Main.cpp
        lib/Thruster_Commander.cpp
        lib/Thruster_Commander.h
)

# GTest start
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/d144031940543e15423a25ae5a8a74141044862f.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
        run_tests
        testing/Command_Interpreter_Testing.cpp
        testing/Thruster_Commander_Testing.cpp
)
target_link_libraries(
        run_tests
        GTest::gtest_main
)
include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master
)
FetchContent_MakeAvailable(yaml-cpp)

target_link_libraries(Propulsion_2024 PUBLIC yaml-cpp::yaml-cpp)
include(GoogleTest)
gtest_discover_tests(run_tests)
#Gtest end

cmake_minimum_required(VERSION 3.16)
project(rocket_roll_sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rocket_roll_sim
    src/main.cpp
)

# Faster builds with compile_commands.json for VS Code intellisense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

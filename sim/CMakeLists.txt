cmake_minimum_required(VERSION 3.16)
project(fc_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT TARGET fc_core)
    add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../core ${CMAKE_CURRENT_BINARY_DIR}/core)
endif()

add_executable(fc_sim main.c profiles.c)
target_link_libraries(fc_sim PRIVATE fc_core)
target_compile_options(fc_sim PRIVATE -Wall -Wextra)

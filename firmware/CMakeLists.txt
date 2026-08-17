cmake_minimum_required(VERSION 3.16)

# Pull in fc_core (the portable core library, see core/) as an extra
# component directory instead of copying it -- core/CMakeLists.txt already
# knows how to register itself as an ESP-IDF component when ESP_PLATFORM is
# set (see the ESP_PLATFORM branch there).
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../core")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(flight_computer)
